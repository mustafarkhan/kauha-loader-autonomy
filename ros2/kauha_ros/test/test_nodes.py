"""ROS 2 node tests: the twin bridge's topics, and the ROS-driven SIL over CAN with fault injection."""
from __future__ import annotations

import json
import math
import random
import threading
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from visualization_msgs.msg import MarkerArray


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Collector(Node):
    def __init__(self, topics: dict, qos=10):
        super().__init__(f"collector_{random.randint(0, 1_000_000)}")
        self.msgs: dict[str, list] = {t: [] for t in topics}
        for t, typ in topics.items():
            self.create_subscription(typ, t, lambda m, t=t: self.msgs[t].append(m), qos)


def test_twin_bridge_publishes_the_shift():
    from kauha_ros.twin_bridge import TwinBridge

    from kauha.demo.shift import ShiftSim
    topics = {"/clock": Clock, "/joint_states": JointState, "/odom": Odometry, "/scan/front_left": LaserScan,
              "/scan/rear_right": LaserScan, "/diagnostics": DiagnosticArray, "/safety/fields": MarkerArray}
    col = Collector(topics, qos=500)
    bridge = TwinBridge(sim=ShiftSim("scripted", 0.5, seed=1))
    bridge.set_parameters([rclpy.parameter.Parameter("start_delay", value=1.0)])
    ex = MultiThreadedExecutor()
    ex.add_node(col)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    bridge.run(max_steps=int(20 / 0.025))         # 20 s of shift
    time.sleep(0.5)
    ex.shutdown()
    m = col.msgs
    assert len(m["/clock"]) > 700 and len(m["/joint_states"]) > 350
    last = m["/clock"][-1].clock
    assert abs(last.sec + last.nanosec * 1e-9 - 20.0) < 0.05
    scan = m["/scan/front_left"][-1]
    assert scan.header.frame_id == "front_left_scanner" and len(scan.ranges) == 276
    assert math.isclose(scan.angle_max - scan.angle_min, math.radians(275), rel_tol=1e-6)
    st = m["/diagnostics"][-1].status[0]
    assert st.name == "safety_plc" and {"state", "cause", "field_set", "speed_limit_m_s"} <= {v.key for v in st.values}
    assert max(abs(o.twist.twist.linear.x) for o in m["/odom"]) > 1.0, "the loader drove"
    assert any(len(a.markers) > 1 for a in m["/safety/fields"]), "field polygons published"
    col.destroy_node()
    bridge.destroy_node()


def _launch_sil(speed: float, person: dict | None, duration: float, kill_driver_at: float | None = None):
    """The product path: ros2 launch kauha_ros sil.launch.py, every node and SIL process in its own process."""
    import subprocess
    cmd = ["ros2", "launch", "kauha_ros", "sil.launch.py", f"speed:={speed}", f"duration:={duration}",
           "person:=" + json.dumps(person or {})]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def _kill_executable(suffix: str) -> int:
    """SIGKILL processes whose program (argv[0], or the script run by an interpreter) ends with suffix.
    Matching on argv only, so shells that merely mention the name are not hit."""
    import os
    import signal
    n = 0
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            argv = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
        except OSError:
            continue
        progs = [a.decode(errors="ignore") for a in argv[:2]]
        if any(p.endswith(suffix) for p in progs):
            os.kill(int(pid), signal.SIGKILL)
            n += 1
    return n


def _states(col):
    out = []
    for a in col.msgs["/diagnostics"]:
        st = a.status[0]
        if st.name != "safety_plc (CAN)":
            continue
        vals = {v.key: v.value for v in st.values}
        out.append((vals["state"], int(vals["cause_bits"]), int(vals["drive_withheld"])))
    return out


@pytest.mark.parametrize("case", ["person", "driver_dies"])
def test_ros_autonomy_drives_the_sil_machine_and_the_plc_stops_it(case):
    """ROS 2 /cmd_drive -> CAN bridge -> DriveCmd (E2E) -> gateway; the safety PLC (own process) stops the
    machine for a person ahead, or when the ROS driver dies (the bridge withholds commands -> PLC watchdog)."""
    import os
    import signal
    person = {"t": 10.0, "ahead": 8.0} if case == "person" else None
    proc = _launch_sil(2.0, person, duration=26.0)
    col = Collector({"/machine/odom": Odometry, "/diagnostics": DiagnosticArray})
    ex = MultiThreadedExecutor()
    ex.add_node(col)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        def wait(cond, timeout):
            end = time.monotonic() + timeout
            while time.monotonic() < end and not cond():
                time.sleep(0.05)
            return cond()

        def speed():
            odom = col.msgs["/machine/odom"]
            return abs(odom[-1].twist.twist.linear.x) if odom else 0.0
        assert wait(lambda: speed() > 1.5, 14.0), f"ROS commands reached the machine over CAN (v={speed()})"
        n0 = len(_states(col))        # ignore the fail-safe stop at power-up, before the supervisor's reset
        withheld0 = _states(col)[-1][2]
        if case == "driver_dies":
            _kill_executable("kauha_ros/straight_driver")
        want = 1 << 0 if case == "person" else 1 << 5          # FIELD_FRONT / WATCHDOG
        stopped = lambda: any(s == "PROTECTIVE_STOP" and c & want for s, c, _ in _states(col)[n0:])  # noqa: E731
        assert wait(stopped, 16.0), _states(col)[-5:]
        assert wait(lambda: speed() < 0.05, 3.0), f"machine at standstill within 3 s of the stop (v={speed()})"
        states = _states(col)
        if case == "driver_dies":
            assert states[-1][2] > withheld0, "the bridge stopped forwarding stale commands"
    finally:
        ex.shutdown()
        col.destroy_node()
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=20)
        except Exception:
            os.killpg(proc.pid, signal.SIGKILL)
