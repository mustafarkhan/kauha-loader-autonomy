"""ROS 2 bridge to the KAUHA digital twin.

Runs the full twin (loader, site, people, ray-cast safety scanners, CAN, safety PLC,
autonomy) and publishes its state on standard ROS 2 interfaces, on simulated time:

  /clock                      rosgraph_msgs/Clock          every 25 ms twin step
  /joint_states               sensor_msgs/JointState       articulation, boom, bucket (20 Hz)
  /tf                         map -> base_link             (20 Hz; robot_state_publisher does the rest)
  /odom                       nav_msgs/Odometry            true pose and speed (20 Hz)
  /localization/odom          nav_msgs/Odometry            the autonomy's EKF estimate (20 Hz)
  /scan/<scanner>             sensor_msgs/LaserScan        each safety scanner's ray-cast ranges (20 Hz)
  /diagnostics                diagnostic_msgs/DiagnosticArray   safety PLC state, cause, field set (20 Hz)
  /safety/fields              visualization_msgs/MarkerArray    active protective and warning fields (5 Hz)
  /site/markers               visualization_msgs/MarkerArray    walls, zones, stockpiles, truck, posts (1 Hz, latched)
  /people/markers             visualization_msgs/MarkerArray    people and unmapped obstacles (5 Hz)
  /kpi                        diagnostic_msgs/DiagnosticArray   shift KPIs when the run ends (latched)

Parameters: controller (learned|scripted), seed, minutes, real_time_factor (0 = as fast as possible),
start_delay (wall seconds before the twin starts, so late subscribers miss nothing).
"""
from __future__ import annotations

import time
from dataclasses import asdict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from kauha_ros import convert as cv

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class TwinBridge(Node):
    def __init__(self, sim=None) -> None:
        super().__init__("kauha_twin")
        self.declare_parameter("controller", "learned")
        self.declare_parameter("seed", 1)
        self.declare_parameter("minutes", 60.0)
        self.declare_parameter("real_time_factor", 1.0)
        self.declare_parameter("start_delay", 2.0)      # wall seconds for subscribers (e.g. a recorder) to join
        if sim is None:
            from kauha.demo.shift import ShiftSim
            p = self.get_parameter
            sim = ShiftSim(p("controller").value, float(p("minutes").value), int(p("seed").value))
        self.sim = sim
        self.rtf = float(self.get_parameter("real_time_factor").value)
        self.minutes = sim.minutes
        from kauha.sim.sensors import FRONT_MOUNTS, REAR_MOUNTS
        self.mounts = {m.name: m for m in FRONT_MOUNTS + REAR_MOUNTS}

        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_loc = self.create_publisher(Odometry, "/localization/odom", 10)
        self.pub_scan = {n: self.create_publisher(LaserScan, f"/scan/{n}", 10) for n in self.mounts}
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.pub_fields = self.create_publisher(MarkerArray, "/safety/fields", 10)
        self.pub_people = self.create_publisher(MarkerArray, "/people/markers", 10)
        self.pub_site = self.create_publisher(MarkerArray, "/site/markers", LATCHED)
        self.pub_kpi = self.create_publisher(DiagnosticArray, "/kpi", LATCHED)
        self.tf = TransformBroadcaster(self)
        self.steps = 0
        self.done = False
        self._t_wall0 = None

    # ------------------------------------------------------------- stepping
    def run(self, max_steps: int | None = None) -> None:
        """Step the twin and publish. Paced by real_time_factor; returns when the shift ends."""
        from kauha.demo.shift import SIM_DT
        n = int(self.minutes * 60 / SIM_DT)
        end = time.monotonic() + float(self.get_parameter("start_delay").value)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._t_wall0 = time.monotonic()
        while rclpy.ok() and self.steps < n and (max_steps is None or self.steps < max_steps):
            self.sim.step()
            self.steps += 1
            self.publish(self.sim.t)
            if self.rtf > 0:
                lag = self.sim.t / self.rtf - (time.monotonic() - self._t_wall0)
                if lag > 0:
                    time.sleep(lag)
        if self.steps >= n:
            self.finish()

    def publish(self, t: float) -> None:
        sim, k = self.sim, self.steps
        self.pub_clock.publish(Clock(clock=cv.stamp(t)))
        if k % 2 == 0:
            s = sim.loader.state
            self.pub_js.publish(cv.joint_state(t, s.gamma, s.boom, s.bucket))
            self.tf.sendTransform(cv.base_transform(t, s.x, s.y, s.theta))
            self.pub_odom.publish(cv.odometry(t, s.x, s.y, s.theta, s.v))
            ex = sim.auto.ekf.x
            self.pub_loc.publish(cv.odometry(t, ex[0], ex[1], ex[2], s.v))
            for name, rng in sim.scanners.last_ranges.items():
                m = self.mounts[name]
                self.pub_scan[name].publish(cv.laser_scan(t, name, rng, m.fov, m.max_range))
            if sim.plc_out is not None:
                fs = sim.plc.sets[sim.plc.field_set]
                arr = DiagnosticArray(header=Header(stamp=cv.stamp(t)),
                                      status=[cv.safety_status(sim.plc_out, fs.name)])
                self.pub_diag.publish(arr)
        if k % 8 == 0:
            self.pub_fields.publish(self.field_markers(t))
            self.pub_people.publish(self.people_markers(t))
        if k % 40 == 1:
            self.pub_site.publish(self.site_markers(t))

    # ------------------------------------------------------------- markers
    def field_markers(self, t: float) -> MarkerArray:
        fs = self.sim.plc.sets[self.sim.plc.field_set]
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL, ns="fields")])
        mid = 0

        def add(frame, pts, kind):
            nonlocal mid
            if pts:
                out.markers.append(cv.polygon_marker(t, frame, "fields", mid, pts, kind))
                mid += 1
        if fs.direction >= 0:
            add("base_link", fs.protective, "protective")
            add("base_link", fs.warning, "warning")
        if fs.direction <= 0:
            add("rear_axle", cv.flip_rear(fs.protective_for("rear")), "protective")
            add("rear_axle", cv.flip_rear(fs.warning_for("rear")), "warning")
        for side in (fs.side, [(a, -b) for a, b in fs.side]):
            add("base_link", side, "protective")
        for side in (fs.side_rear, [(a, -b) for a, b in fs.side_rear]):
            add("rear_axle", cv.flip_rear(side), "protective")
        return out

    def people_markers(self, t: float) -> MarkerArray:
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL, ns="people")])
        for i, (_, x, y) in enumerate(self.sim.site.people_at(t)):
            out.markers.append(cv.cylinder_marker(t, "people", i, x, y, 0.25, 1.8, "person"))
        for j, o in enumerate(self.sim.active_obstacles()):
            out.markers.append(cv.box_marker(t, "people", 100 + j, o["x"], o["y"], 0.6 * o["r"], 2 * o["r"],
                                             2 * o["r"], 1.2 * o["r"], "block"))
        return out

    def site_markers(self, t: float) -> MarkerArray:
        site = self.sim.site
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL, ns="site")])
        mid = 0

        def nxt():
            nonlocal mid
            mid += 1
            return mid
        for x0, y0, x1, y1 in site.walls:
            out.markers.append(cv.polygon_marker(t, "map", "site", nxt(), [(x0, y0), (x1, y1)], "wall", z=0.45,
                                                 width=0.6))
        for z in site.dig_zones.values():
            out.markers.append(cv.polygon_marker(t, "map", "site", nxt(), z, "zone", z=0.03))
        out.markers.append(cv.polygon_marker(t, "map", "site", nxt(), site.dock_zone, "dock", z=0.03))
        for name, pile in site.piles.items():
            for sl in pile.slots:
                length = pile.back_x - sl.toe_x
                out.markers.append(cv.box_marker(t, "site", nxt(), sl.toe_x + length / 2, sl.y_c, pile.height / 2,
                                                 length, sl.width - 0.1, pile.height, f"pile_{name}"))
        tr = site.truck
        if tr.present:
            out.markers.append(cv.box_marker(t, "site", nxt(), tr.cx, tr.cy, 1.6, tr.length, tr.width, 2.4,
                                             "truck", tr.heading))
        for lx, ly in site.landmarks:
            out.markers.append(cv.cylinder_marker(t, "site", nxt(), lx, ly, 0.08, 1.6, "post"))
        return out

    def finish(self) -> None:
        k = asdict(self.sim.kpis(time.monotonic() - (self._t_wall0 or time.monotonic())))
        vals = [KeyValue(key=key, value=str(v)) for key, v in k.items() if not isinstance(v, (dict, list))]
        self.pub_kpi.publish(DiagnosticArray(header=Header(stamp=cv.stamp(self.sim.t)), status=[
            DiagnosticStatus(level=DiagnosticStatus.OK, name="kauha_shift_kpis", values=vals)]))
        self.get_logger().info(f"shift done: {k['tonnes']} t, {k['cycles']} cycles, {k['interventions']} "
                               f"interventions, closest person {k['min_person_distance_moving_m']} m")
        self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinBridge()
    try:
        node.run()
        # keep the latched KPI and site topics available for late subscribers (e.g. a recorder)
        end = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
