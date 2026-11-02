"""Pure conversions from the twin's state to ROS 2 messages (no node, easy to test)."""
from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

# Mounts in the URDF frames (link, yaw in the link). The twin's rear mounts are given in a
# backward-facing rear frame; the URDF rear_axle frame faces forward.
SCANNER_LINKS = {"front_left": "front_left_scanner", "front_right": "front_right_scanner",
                 "rear_left": "rear_left_scanner", "rear_right": "rear_right_scanner"}
STATE_NAMES = ["RUN", "WARNING", "PROTECTIVE_STOP", "EMERGENCY_STOP"]
COLORS = {"protective": (0.78, 0.2, 0.17, 0.55), "warning": (0.85, 0.51, 0.12, 0.9), "zone": (0.17, 0.44, 0.68, 0.8),
          "dock": (0.63, 0.15, 0.61, 0.8), "pile_A": (0.73, 0.68, 0.6, 1.0), "pile_B": (0.62, 0.63, 0.64, 1.0),
          "person": (0.17, 0.44, 0.68, 1.0), "truck": (0.17, 0.44, 0.68, 1.0), "wall": (0.6, 0.62, 0.58, 1.0),
          "post": (0.95, 0.95, 0.95, 1.0), "block": (0.12, 0.14, 0.13, 1.0)}


def stamp(t: float) -> Time:
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def yaw_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def color(name: str) -> ColorRGBA:
    r, g, b, a = COLORS[name]
    return ColorRGBA(r=r, g=g, b=b, a=a)


def joint_state(t: float, gamma: float, boom: float, bucket: float) -> JointState:
    """Bucket joint: the twin's bucket state is a curl command, drawn as 0.6 * curl relative to level."""
    return JointState(header=Header(stamp=stamp(t)), name=["articulation", "boom", "bucket"],
                      position=[float(gamma), float(boom), float(0.6 * bucket - boom)])


def odometry(t: float, x: float, y: float, th: float, v: float, gamma_rate: float = 0.0) -> Odometry:
    o = Odometry(header=Header(stamp=stamp(t), frame_id="map"), child_frame_id="base_link")
    o.pose.pose.position.x, o.pose.pose.position.y = float(x), float(y)
    o.pose.pose.orientation = yaw_quat(th)
    o.twist.twist.linear.x = float(v)
    return o


def base_transform(t: float, x: float, y: float, th: float) -> TransformStamped:
    tf = TransformStamped(header=Header(stamp=stamp(t), frame_id="map"), child_frame_id="base_link")
    tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
    tf.transform.rotation = yaw_quat(th)
    return tf


def laser_scan(t: float, name: str, ranges, fov: float, max_range: float) -> LaserScan:
    n = len(ranges)
    msg = LaserScan(header=Header(stamp=stamp(t), frame_id=SCANNER_LINKS[name]))
    msg.angle_min, msg.angle_max = -fov / 2, fov / 2
    msg.angle_increment = fov / (n - 1)
    msg.scan_time = 0.05
    msg.range_min, msg.range_max = 0.1, float(max_range)
    msg.ranges = [float(r) if r < max_range else math.inf for r in ranges]
    return msg


def safety_status(out, field_name: str, name: str = "safety_plc") -> DiagnosticStatus:
    """The PLC outputs as a diagnostic (state, cause, field set, limits, relay outputs)."""
    level = DiagnosticStatus.OK if int(out.state) == 0 else DiagnosticStatus.WARN if int(out.state) == 1 \
        else DiagnosticStatus.ERROR
    kv = {"state": STATE_NAMES[int(out.state)], "cause": out.cause.name or "NONE", "field_set": field_name,
          "speed_limit_m_s": f"{out.speed_limit:.2f}", "brake_release": str(bool(out.brake_release)).lower(),
          "implement_enable": str(bool(out.implement_enable)).lower()}
    return DiagnosticStatus(level=level, name=name, message=kv["state"], hardware_id="kauha_safety_plc",
                            values=[KeyValue(key=k, value=v) for k, v in kv.items()])


def polygon_marker(t: float, frame: str, ns: str, mid: int, pts, kind: str, z: float = 0.05,
                   width: float = 0.08) -> Marker:
    m = Marker(header=Header(stamp=stamp(t), frame_id=frame), ns=ns, id=mid, type=Marker.LINE_STRIP,
               action=Marker.ADD)
    m.scale.x = width
    m.color = color(kind)
    m.points = [Point(x=float(a), y=float(b), z=z) for a, b in list(pts) + [pts[0]]] if pts else []
    return m


def box_marker(t: float, ns: str, mid: int, cx: float, cy: float, cz: float, sx: float, sy: float, sz: float,
               kind: str, yaw: float = 0.0, frame: str = "map") -> Marker:
    m = Marker(header=Header(stamp=stamp(t), frame_id=frame), ns=ns, id=mid, type=Marker.CUBE, action=Marker.ADD)
    m.pose.position = Point(x=float(cx), y=float(cy), z=float(cz))
    m.pose.orientation = yaw_quat(yaw)
    m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
    m.color = color(kind)
    return m


def cylinder_marker(t: float, ns: str, mid: int, x: float, y: float, r: float, h: float, kind: str) -> Marker:
    m = Marker(header=Header(stamp=stamp(t), frame_id="map"), ns=ns, id=mid, type=Marker.CYLINDER,
               action=Marker.ADD)
    m.pose.position = Point(x=float(x), y=float(y), z=h / 2)
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = 2 * float(r)
    m.scale.z = float(h)
    m.color = color(kind)
    return m


def flip_rear(pts):
    """Rear-frame polygons in the twin face backwards; the URDF rear_axle frame faces forward."""
    return [(-a, -b) for a, b in pts]
