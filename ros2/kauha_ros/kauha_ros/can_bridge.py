"""ROS 2 <-> KAUHA machine bus bridge (runs on the autonomy computer).

On the machine the safety PLC and the drive-by-wire gateway live on CAN, not on ROS:
ROS 2 is only the autonomy computer's middleware. This node is the autonomy
computer's CAN interface (the role ros2_socketcan plays on hardware; here the bus
is python-can, socketcan or the SIL's UDP-multicast bus).

  /cmd_drive        ackermann_msgs/AckermannDriveStamped  speed (m/s), steering_angle = articulation (rad)
  /cmd_implement    sensor_msgs/JointState                velocity of "boom" and "bucket" (rad/s)
  /machine/joint_states, /machine/odom                    decoded MachineState1 (speed, articulation, boom)
  /diagnostics                                            decoded SafetyStatus + bus statistics

The bridge never keeps a stale command alive: if /cmd_drive has not arrived for
cmd_timeout (0.1 s), it stops sending DriveCmd, so the safety PLC's watchdog
(SR-06, 200 ms) and the gateway's timeout see the autonomy's failure as it is.
"""
from __future__ import annotations

import time

import can
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from kauha.can.codec import Codec

STATE = ["RUN", "WARNING", "PROTECTIVE_STOP", "EMERGENCY_STOP"]


class CanBridge(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("kauha_can_bridge", **node_kwargs)
        self.declare_parameter("interface", "udp_multicast")
        self.declare_parameter("channel", "239.74.0.1")
        self.declare_parameter("mode", 1)                 # 1 AUTO, 2 TELEOP
        self.declare_parameter("cmd_timeout", 0.1)
        p = self.get_parameter
        self.bus = can.Bus(interface=p("interface").value, channel=p("channel").value, receive_own_messages=False)
        self.codec = Codec()
        self.rx_status = self.codec.receiver("SafetyStatus", 0.1)
        self.mode = int(p("mode").value)
        self.cmd_timeout = float(p("cmd_timeout").value)
        self.drive: AckermannDriveStamped | None = None
        self.drive_t = -1e9
        self.implement = (0.0, 0.0)
        self.stats = {"frames_rx": 0, "e2e_rejected": 0, "drive_sent": 0, "drive_withheld": 0}
        self.create_subscription(AckermannDriveStamped, "/cmd_drive", self.on_drive, 10)
        self.create_subscription(JointState, "/cmd_implement", self.on_implement, 10)
        self.pub_js = self.create_publisher(JointState, "/machine/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "/machine/odom", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(0.05, self.send_commands)       # 20 Hz, as DriveCmd/ImplementCmd in the DBC
        self.create_timer(0.005, self.receive)

    def on_drive(self, msg: AckermannDriveStamped) -> None:
        self.drive, self.drive_t = msg, time.monotonic()

    def on_implement(self, msg: JointState) -> None:
        v = dict(zip(msg.name, msg.velocity))
        self.implement = (float(v.get("boom", 0.0)), float(v.get("bucket", 0.0)))

    def _send(self, name: str, sig: dict) -> None:
        fid, data = self.codec.encode(name, sig)
        self.bus.send(can.Message(arbitration_id=fid, data=data, is_extended_id=True))

    def send_commands(self) -> None:
        fresh = self.drive is not None and time.monotonic() - self.drive_t <= self.cmd_timeout
        if not fresh:
            self.stats["drive_withheld"] += 1          # let the PLC watchdog see the silence
            return
        d = self.drive.drive
        self._send("DriveCmd", {"SpeedReq": float(max(-5.9, min(5.9, d.speed))),
                                "ArticulationReq": float(max(-0.79, min(0.79, d.steering_angle))),
                                "Mode": self.mode, "BrakeReq": 0})
        self._send("ImplementCmd", {"BoomRate": self.implement[0], "BucketRate": self.implement[1], "Hold": 0})
        self.stats["drive_sent"] += 1

    def receive(self) -> None:
        while (m := self.bus.recv(timeout=0.0)) is not None:
            self.stats["frames_rx"] += 1
            try:
                name, sig = self.codec.decode(m.arbitration_id, bytes(m.data))
            except KeyError:
                continue
            now = self.get_clock().now().to_msg()
            if name == "MachineState1":
                self.pub_js.publish(JointState(header=Header(stamp=now), name=["articulation", "boom"],
                                               position=[float(sig["Articulation"]), float(sig["Boom"])]))
                o = Odometry(header=Header(stamp=now, frame_id="odom"), child_frame_id="base_link")
                o.twist.twist.linear.x = float(sig["Speed"])
                self.pub_odom.publish(o)
            elif name == "SafetyStatus":
                ok = self.rx_status.check(bytes(m.data), time.monotonic()).value.startswith("ok")
                if not ok:
                    self.stats["e2e_rejected"] += 1
                    continue
                st = int(sig["State"])
                level = DiagnosticStatus.OK if st == 0 else DiagnosticStatus.WARN if st == 1 else DiagnosticStatus.ERROR
                vals = {"state": STATE[st], "cause_bits": str(int(sig["Cause"])),
                        "field_set": str(int(sig["ActiveFieldSet"])), "speed_limit_m_s": f"{sig['SpeedLimit']:.2f}",
                        "reset_pending": str(int(sig["ResetPending"])), **{k: str(v) for k, v in self.stats.items()}}
                self.pub_diag.publish(DiagnosticArray(header=Header(stamp=now), status=[DiagnosticStatus(
                    level=level, name="safety_plc (CAN)", message=STATE[st], hardware_id="kauha_safety_plc",
                    values=[KeyValue(key=k, value=v) for k, v in vals.items()])]))

    def destroy_node(self) -> None:
        self.bus.shutdown()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CanBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
