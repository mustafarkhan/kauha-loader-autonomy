"""Parametric CAD of the KAUHA retrofit kit on a simplified 15 t articulated loader.

    python -m hardware.cad.kauha_cad            # build everything into hardware/cad/export/
    python -m hardware.cad.kauha_cad --check    # interference and position checks only

Concept-level geometry for layout, clearance and visualisation, not manufacturing
drawings. Every kit position comes from the same numbers the twin and the URDF use
(kauha.config, kauha.sim.sensors, docs/hardware/sensor_mounting.md), and --check
verifies that.

Frames (as the twin): x forward, y left, z up, origin = front axle centre on the
ground. The rear frame hangs off the articulation joint 1.6 m behind the front axle;
the boom pivots at (0.9, 0, 1.95); the bucket pivots at the boom tip.

Outputs, all in hardware/cad/export/:
  step/<part>.step       each kit part, in its own mounting frame
  stl/<part>.stl         the same, for 3D-printed fit-check mock-ups
  kauha_assembly.step    the full assembly in the carry pose
  kauha_kit.glb          tessellated scene graph with joint nodes, for the web viewer
  parts.json             part list: group, channel (safety / autonomy / machine), BOM ref
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq

from kauha.config import load_machine
from kauha.sim.sensors import FRONT_MOUNTS, REAR_MOUNTS

C = load_machine()
OUT = Path(__file__).resolve().parent / "export"
JOINT_X = -C.lf                      # articulation joint, front-frame coordinates
REAR_AXLE_X = -(C.lf + C.lr)
TYRE_R, TYRE_W = 0.78, 0.6
TRACK = C.track_width                # wheel centre spacing
SCAN_Z = 0.5                         # safety scanner plane height


@dataclass
class Part:
    name: str
    group: str               # front | rear | boom | bucket (the joint node it rides on)
    channel: str             # machine | safety | autonomy
    shape: cq.Workplane      # in the group's local frame
    bom: str = ""
    note: str = ""
    color: str = ""
    kit: bool = False


PALETTE = {"machine": "#e2a312", "machine_dark": "#3a3326", "tyre": "#1f2122", "rim": "#8c8a83",
           "glass": "#9fc3d1", "safety": "#d43b2a", "autonomy": "#2563c9", "steel": "#5d636b"}


# --------------------------------------------------------------------------- helpers
def box(lx, ly, lz, x=0.0, y=0.0, z=0.0) -> cq.Workplane:
    return cq.Workplane("XY").box(lx, ly, lz).translate((x, y, z))


def cyl_y(r, length, x=0.0, y=0.0, z=0.0) -> cq.Workplane:
    """Cylinder with its axis along y, centred at (x, y, z)."""
    return cq.Workplane("XZ").circle(r).extrude(length / 2, both=True).translate((x, y, z))


def cyl_z(r, h, x=0.0, y=0.0, z=0.0) -> cq.Workplane:
    """Cylinder with its axis along z, base at z."""
    return cq.Workplane("XY").circle(r).extrude(h).translate((x, y, z))


def tube(p0, p1, w=0.08, h=0.08) -> cq.Workplane:
    """Rectangular tube between two points (any direction)."""
    v = cq.Vector(*p1) - cq.Vector(*p0)
    length = v.Length
    mid = (cq.Vector(*p0) + cq.Vector(*p1)) * 0.5
    t = cq.Workplane("XY").box(length, w, h)
    yaw = math.degrees(math.atan2(v.y, v.x))
    pitch = math.degrees(math.atan2(v.z, math.hypot(v.x, v.y)))
    return t.rotate((0, 0, 0), (0, 1, 0), -pitch).rotate((0, 0, 0), (0, 0, 1), yaw).translate(mid.toTuple())


def tyre(x, y, z=TYRE_R) -> cq.Workplane:
    t = cyl_y(TYRE_R, TYRE_W, x, y, z)
    for k in range(-2, 3):            # tread grooves (rings)
        t = t.cut(cyl_y(TYRE_R + 0.01, 0.025, x, y + k * 0.11, z).cut(cyl_y(TYRE_R - 0.035, 0.03, x, y + k * 0.11, z)))
    hub = cyl_y(0.42, TYRE_W + 0.02, x, y, z)
    return t.cut(cyl_y(0.42, TYRE_W + 0.1, x, y, z)).union(hub.cut(cyl_y(0.18, TYRE_W + 0.1, x, y, z)))


def rim(x, y, z=TYRE_R) -> cq.Workplane:
    return cyl_y(0.42, TYRE_W - 0.05, x, y, z).cut(cyl_y(0.34, TYRE_W, x, y, z)).union(
        cyl_y(0.2, TYRE_W + 0.06, x, y, z))


# --------------------------------------------------------------------------- base machine
def machine_parts() -> list[Part]:
    p: list[Part] = []
    hw = TRACK / 2
    # Front frame (front-axle frame): chassis beam, nose, fenders, axle, wheels.
    chassis = box(2.9, 1.2, 0.75, -0.35, 0, 1.0).edges("|X").fillet(0.08)
    nose = box(0.25, 1.5, 0.9, 1.0, 0, 1.0)
    towers = box(0.5, 0.25, 1.35, 0.85, 0.62, 1.32).union(box(0.5, 0.25, 1.35, 0.85, -0.62, 1.32))
    hitch = box(0.4, 0.6, 0.5, JOINT_X + 0.25, 0, 1.0)
    p.append(Part("front_frame", "front", "machine", chassis.union(nose).union(towers).union(hitch),
                  note="Front frame with boom towers and articulation hitch"))
    axle = cyl_y(0.16, TRACK - 0.2, 0, 0, TYRE_R).union(box(0.5, 0.7, 0.35, 0, 0, TYRE_R))
    p.append(Part("front_axle", "front", "machine", axle, color=PALETTE["machine_dark"]))
    for side, name in ((1, "left"), (-1, "right")):
        p.append(Part(f"front_tyre_{name}", "front", "machine", tyre(0, side * hw), color=PALETTE["tyre"]))
        p.append(Part(f"front_rim_{name}", "front", "machine", rim(0, side * hw), color=PALETTE["rim"]))
        fender = box(1.5, 0.66, 0.06, 0.05, side * hw, 1.68).union(box(0.06, 0.66, 0.35, -0.68, side * hw, 1.5))
        p.append(Part(f"front_fender_{name}", "front", "machine", fender))
    # Lift cylinders (frame to boom, fixed visual at the carry pose).
    for side in (1, -1):
        cylb = tube((0.55, side * 0.55, 1.05), (1.75, side * 0.62, 1.55), 0.16, 0.16)
        p.append(Part(f"lift_cylinder_{'left' if side > 0 else 'right'}", "front", "machine", cylb,
                      color=PALETTE["steel"], note="Shown at the carry pose; not animated"))

    # Rear frame (local origin at the articulation joint, x forward like the front frame).
    ra = REAR_AXLE_X - JOINT_X           # rear axle in joint coordinates (-1.6)
    hitch_r = box(0.5, 0.9, 0.6, -0.3, 0, 1.0)
    deck = box(2.6, 1.6, 0.25, ra - 0.4, 0, 1.2)
    engine = (box(2.1, 1.45, 0.9, ra - 0.75, 0, 1.75).edges("|Y and >Z").fillet(0.15))
    grille = box(0.06, 1.3, 0.75, ra - 1.82, 0, 1.75)
    cw = box(0.45, 2.2, 1.15, ra - 1.95, 0, 1.25).edges("|Z").fillet(0.12)
    stack = cyl_z(0.07, 0.75, ra - 0.2, -0.45, 2.2)
    p.append(Part("rear_frame", "rear", "machine", hitch_r.union(deck).union(engine).union(stack),
                  note="Rear frame, engine hood and exhaust"))
    p.append(Part("counterweight", "rear", "machine", cw.union(grille), color=PALETTE["machine_dark"]))
    cab_x = ra + 0.9
    posts = None
    for sx in (-0.55, 0.55):
        for sy in (-0.65, 0.65):
            post = box(0.08, 0.08, 1.4, cab_x + sx, sy, 2.38)
            posts = post if posts is None else posts.union(post)
    roof = box(1.35, 1.55, 0.12, cab_x, 0, 3.12)
    base = box(1.3, 1.45, 0.35, cab_x, 0, 1.5)
    p.append(Part("cab_frame", "rear", "machine", posts.union(roof).union(base), note="Cab with ROPS posts"))
    p.append(Part("cab_glass", "rear", "machine", box(1.1, 1.25, 1.3, cab_x, 0, 2.38), color=PALETTE["glass"]))
    steps = None
    for k in range(3):
        s = box(0.35, 0.12, 0.04, cab_x - 0.2, 0.78, 0.6 + 0.35 * k)
        steps = s if steps is None else steps.union(s)
    p.append(Part("access_steps", "rear", "machine", steps, color=PALETTE["steel"]))
    p.append(Part("rear_axle", "rear", "machine", cyl_y(0.16, TRACK - 0.2, ra, 0, TYRE_R)
                  .union(box(0.5, 0.7, 0.35, ra, 0, TYRE_R)), color=PALETTE["machine_dark"]))
    for side, name in ((1, "left"), (-1, "right")):
        p.append(Part(f"rear_tyre_{name}", "rear", "machine", tyre(ra, side * hw), color=PALETTE["tyre"]))
        p.append(Part(f"rear_rim_{name}", "rear", "machine", rim(ra, side * hw), color=PALETTE["rim"]))
        p.append(Part(f"rear_fender_{name}", "rear", "machine",
                      box(1.4, 0.66, 0.06, ra - 0.1, side * hw, 1.66)))

    # Boom (local origin at the boom pivot, along +x at boom angle 0).
    arms = None
    for side in (1, -1):
        a = box(C.boom_length, 0.2, 0.28, C.boom_length / 2, side * 0.75, 0).edges("|Y").fillet(0.06)
        arms = a if arms is None else arms.union(a)
    cross = cyl_y(0.12, 1.5, C.boom_length * 0.55, 0, 0)
    pins = cyl_y(0.07, 1.9, 0, 0, 0).union(cyl_y(0.07, 1.9, C.boom_length, 0, 0))
    tilt = tube((C.boom_length * 0.3, 0, 0.25), (C.boom_length * 0.85, 0, 0.3), 0.15, 0.15)
    p.append(Part("boom", "boom", "machine", arms.union(cross).union(pins)))
    p.append(Part("tilt_cylinder", "boom", "machine", tilt, color=PALETTE["steel"]))

    # Bucket (local origin at the bucket pivot; cutting edge 1.3 m ahead, 0.45 m below at bucket = 0,
    # as kauha.kinematics.bucket_tip_local).
    bw = C.bucket_width
    profile = (cq.Workplane("XZ").moveTo(-0.08, 0.55).lineTo(0.35, 0.78).lineTo(0.42, 0.7).lineTo(0.02, 0.48)
               .lineTo(0.02, -0.2).threePointArc((0.12, -0.42), (0.35, -0.5)).lineTo(C.bucket_length, -0.45)
               .lineTo(C.bucket_length, -0.52).lineTo(0.3, -0.6).threePointArc((0.0, -0.5), (-0.08, -0.22)).close())
    shell = profile.extrude(bw / 2, both=True)
    def side_prof():
        return (cq.Workplane("XZ").moveTo(-0.08, 0.55).lineTo(0.38, 0.75).lineTo(C.bucket_length, -0.45)
                .lineTo(0.3, -0.6).threePointArc((0.0, -0.5), (-0.08, -0.22)).close())
    sides = None
    for side in (1, -1):
        s = side_prof().extrude(0.03, both=True).translate((0, side * (bw / 2 - 0.03), 0))
        sides = s if sides is None else sides.union(s)
    lugs = None
    for side in (1, -1):
        lug = box(0.25, 0.08, 0.3, 0.0, side * 0.75, 0.05)
        lugs = lug if lugs is None else lugs.union(lug)
    edge = box(0.12, bw, 0.04, C.bucket_length - 0.04, 0, -0.485)
    p.append(Part("bucket", "bucket", "machine", shell.union(sides).union(lugs), color=PALETTE["machine_dark"],
                  note=f"{C.bucket_volume:.0f} m³, {bw:.1f} m wide"))
    p.append(Part("cutting_edge", "bucket", "machine", edge, color=PALETTE["steel"]))
    return p


# --------------------------------------------------------------------------- retrofit kit
def scanner_body() -> cq.Workplane:
    """outdoorScan3-sized scanner (about 0.15 x 0.15 x 0.19 m) with its optics window facing +x."""
    housing = box(0.15, 0.15, 0.19, -0.02, 0, 0).edges("|Z").fillet(0.02)
    window = cyl_z(0.068, 0.07, 0.03, 0, -0.06)
    hood = box(0.05, 0.17, 0.02, 0.04, 0, 0.1)
    return housing.union(window).union(hood)


def place(shape: cq.Workplane, x, y, z, yaw_rad) -> cq.Workplane:
    return shape.rotate((0, 0, 0), (0, 0, 1), math.degrees(yaw_rad)).translate((x, y, z))


def kit_parts() -> list[Part]:
    p: list[Part] = []
    # --- Safety scanners (A2/A3), exactly at the twin's mounts.
    for m in FRONT_MOUNTS:
        p.append(Part(f"scanner_{m.name}", "front", "safety", place(scanner_body(), m.x, m.y, SCAN_Z, m.heading),
                      bom="A2", note="SICK outdoorScan3, 275°, yaw ±45° outward", kit=True))
        s = 1 if m.y > 0 else -1
        arm = tube((1.12, s * 0.62, 0.95), (m.x - 0.02, m.y - s * 0.02, SCAN_Z + 0.16), 0.09, 0.09)
        drop = box(0.1, 0.1, 0.06, m.x - 0.02, m.y - s * 0.02, SCAN_Z + 0.14)
        plate = box(0.03, 0.3, 0.3, 1.13, s * 0.62, 0.95)
        corner = (m.x + 0.14, m.y + s * 0.14)
        hoop = tube((*corner, SCAN_Z + 0.13), (*corner, SCAN_Z - 0.12), 0.03, 0.03)
        hoop = hoop.union(tube((m.x - 0.02, m.y - s * 0.02, SCAN_Z + 0.13), (m.x + 0.14, m.y + s * 0.14, SCAN_Z + 0.13),
                               0.03, 0.03))
        p.append(Part(f"bracket_{m.name}", "front", "safety", arm.union(drop).union(plate).union(hoop),
                      bom="A2", note="Bolt-on arm from the frame nose with a bash hoop; keeps the scanner "
                                     "ahead of the tyre at 0.5 m", color=PALETTE["steel"], kit=True))
    ra = REAR_AXLE_X - JOINT_X
    for m in REAR_MOUNTS:
        # sensors.py rear mounts are in the rear frame with x backwards; convert to joint coordinates.
        x, y, yaw = ra - m.x, -m.y, math.pi - m.heading
        p.append(Part(f"scanner_{m.name}", "rear", "safety", place(scanner_body(), x, y, SCAN_Z, yaw),
                      bom="A3", note="SICK outdoorScan3 under the counterweight corner, 45° outward", kit=True))
        s = 1 if y > 0 else -1
        hanger = box(0.12, 0.12, 0.24, x + 0.02, y - s * 0.02, SCAN_Z + 0.2)
        guard = tube((x - 0.15, y + s * 0.13, SCAN_Z + 0.12), (x - 0.15, y + s * 0.13, SCAN_Z - 0.12), 0.03, 0.03)
        p.append(Part(f"bracket_{m.name}", "rear", "safety", hanger.union(guard), bom="A3",
                      color=PALETTE["steel"], kit=True))
    # --- Safety controller, relays, e-stops, safety GNSS, beacon (A1, A6-A9, A11).
    plc = box(0.45, 0.35, 0.22, ra - 0.55, -0.42, 1.44).edges("|Z").fillet(0.02)
    p.append(Part("safety_plc_enclosure", "rear", "safety", plc, bom="A1 + A9",
                  note="Safety PLC (PL d) and force-guided relays, IP67 enclosure on the rear deck", kit=True))
    radio = box(0.25, 0.18, 0.12, ra - 1.2, -0.45, 1.39).union(cyl_z(0.012, 0.35, ra - 1.2, -0.45, 1.45))
    p.append(Part("safety_radio_receiver", "rear", "safety", radio, bom="A7",
                  note="Remote e-stop radio, separate from the data radio", kit=True))
    for side, name in ((1, "left"), (-1, "right")):
        pod = box(0.1, 0.06, 0.14, ra - 0.4, side * 0.76, 1.55)
        mush = cq.Workplane("XZ").circle(0.045).extrude(-side * 0.04).translate((ra - 0.4, side * 0.79, 1.55))
        p.append(Part(f"estop_{name}", "rear", "safety", pod.union(mush), bom="A8",
                      note="Two-channel mushroom e-stop reachable from the ground", kit=True))
    cab_x = ra + 0.9
    p.append(Part("estop_cab", "rear", "safety", cyl_z(0.035, 0.05, cab_x + 0.4, 0.4, 2.05)
                  .union(box(0.09, 0.09, 0.08, cab_x + 0.4, 0.4, 2.01)), bom="A8", kit=True))
    p.append(Part("safety_gnss_antenna", "rear", "safety", cyl_z(0.02, 0.45, ra - 0.5, 0, 2.2)
                  .union(cyl_z(0.075, 0.035, ra - 0.5, 0, 2.65)), bom="A6",
                  note="Independent geofence GNSS, not RTK", kit=True))
    for side in (1, -1):
        bea = cyl_z(0.06, 0.12, cab_x - 0.55, side * 0.6, 3.18)
        p.append(Part(f"beacon_{'left' if side > 0 else 'right'}", "rear", "safety", bea, bom="A11",
                      note="Amber beacon, autonomous-mode warning", kit=True))
    p.append(Part("articulation_sensor", "front", "safety",
                  cyl_z(0.05, 0.08, JOINT_X + 0.05, 0, 1.38).union(box(0.12, 0.08, 0.04, JOINT_X + 0.12, 0, 1.4)),
                  bom="A5", note="Contactless angle sensor on the joint pin (field sector selection)", kit=True))
    for side in (1, -1):
        p.append(Part(f"boom_angle_sensor_{'left' if side > 0 else 'right'}", "front", "safety",
                      cyl_y(0.06, 0.06, C.boom_pivot_forward, side * 0.98, C.boom_pivot_height), bom="A11 (boom)",
                      note="Two independent channels: bucket above 1.0 m enables the far under-bucket field (SR-16)",
                      kit=True))
    p.append(Part("pilot_cutoff_valve", "front", "safety", box(0.16, 0.12, 0.12, -1.0, -0.45, 1.44)
                  .union(cyl_z(0.03, 0.08, -1.0, -0.45, 1.5)), bom="A10",
                  note="Normally closed: no pilot pressure without the PLC", kit=True))
    # --- Autonomy and compute (B, C, D).
    mx = -0.4
    mast = tube((mx, 0, 1.37), (mx, 0, 3.3), 0.12, 0.12).union(
        tube((mx, -0.85, 3.36), (mx, 0.85, 3.36), 0.08, 0.06)).union(
        tube((mx, 0, 1.6), (mx + 0.55, 0, 1.37), 0.06, 0.06)).union(
        tube((mx, 0, 1.6), (mx - 0.55, 0, 1.37), 0.06, 0.06))
    p.append(Part("sensor_mast", "front", "autonomy", mast, color=PALETTE["steel"], kit=True,
                  note="On the front frame, so the lidar turns with the bucket; clears the boom and the cab "
                       "at full articulation"))
    p.append(Part("lidar", "front", "autonomy", cyl_z(0.0435, 0.074, mx, 0, 3.39).union(
        box(0.12, 0.12, 0.02, mx, 0, 3.39)), bom="B1", note="Ouster OS1, 0.5-60 m", kit=True))
    for side, name in ((1, "left"), (-1, "right")):
        p.append(Part(f"gnss_antenna_{name}", "front", "autonomy",
                      cyl_z(0.075, 0.04, mx - 0.2, side * 0.8, 3.39)
                      .union(box(0.25, 0.04, 0.03, mx - 0.1, side * 0.8, 3.39)),
                      bom="B4", note="RTK GNSS, 1.6 m heading baseline", kit=True))
    p.append(Part("radar", "front", "autonomy", box(0.06, 0.14, 0.12, 1.16, 0, 1.0), bom="B2",
                  note="4D radar on the frame nose, under the boom's swept volume (moved from 2.2 m in CAD layout)",
                  kit=True))
    cams = {"front": (cab_x + 0.68, 0, 3.05, 0.0), "rear": (cab_x - 0.68, 0, 3.05, math.pi),
            "left": (cab_x, 0.78, 3.05, math.pi / 2), "right": (cab_x, -0.78, 3.05, -math.pi / 2)}
    for name, (x, y, z, yaw) in cams.items():
        cam = box(0.09, 0.07, 0.07).union(cq.Workplane("YZ").circle(0.022).extrude(0.03).translate((0.045, 0, 0)))
        p.append(Part(f"camera_{name}", "rear", "autonomy", place(cam, x, y, z, yaw), bom="B3",
                      note="Heated GMSL2 camera: person/PPE detection and remote video", kit=True))
    p.append(Part("autonomy_computer", "rear", "autonomy",
                  box(0.5, 0.4, 0.22, ra + 0.15, 0.42, 1.44).edges("|Z").fillet(0.02), bom="C1 + C2 + C4 + C5",
                  note="Jetson AGX Orin, CAN interface, switch and SSD in one IP67 box", kit=True))
    p.append(Part("drive_by_wire_gateway", "rear", "autonomy",
                  box(0.35, 0.3, 0.14, ra + 0.15, -0.42, 1.4).edges("|Z").fillet(0.015), bom="D1",
                  note="Bridges the KAUHA bus to the OEM bus; applies the PLC speed limit", kit=True))
    router = box(0.3, 0.2, 0.1, ra - 1.2, 0.45, 1.38)
    for dy in (-0.06, 0.06):
        router = router.union(cyl_z(0.01, 0.3, ra - 1.25, 0.45 + dy, 1.43))
    p.append(Part("data_radio", "rear", "autonomy", router, bom="C3",
                  note="Private LTE/5G router, two modems", kit=True))
    return p


ALL_GROUPS = ("front", "rear", "boom", "bucket")

# Layout findings from the interference sweep, kept true by tests/hardware/test_cad.py.
LAYOUT_FINDINGS = [
    {"part": "radar", "was": [2.2, 0.0, 1.2], "now": [1.16, 0.0, 1.0],
     "finding": "At the documented position the 4D radar is struck by the boom's tilt cylinder when the boom is "
                "fully lowered. Moved to the frame nose, below the boom's swept volume."},
]


def build_parts() -> list[Part]:
    parts = machine_parts() + kit_parts()
    for pt in parts:
        if not pt.color:
            pt.color = PALETTE[pt.channel]
    return parts


# --------------------------------------------------------------------------- poses
def group_transforms(gamma: float = 0.0, boom: float = -0.2, bucket: float = 0.55) -> dict[str, cq.Location]:
    """World placement of each joint node. URDF conventions: rear frame yaw = -gamma about the joint;
    boom pitch = boom (positive raises); bucket pitch relative to the boom."""
    front = cq.Location(cq.Vector(0, 0, 0))
    rear = front * cq.Location(cq.Vector(JOINT_X, 0, 0)) * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1),
                                                                         -math.degrees(gamma))
    boom_l = front * cq.Location(cq.Vector(C.boom_pivot_forward, 0, C.boom_pivot_height)) * cq.Location(
        cq.Vector(0, 0, 0), cq.Vector(0, -1, 0), math.degrees(boom))
    bucket_l = boom_l * cq.Location(cq.Vector(C.boom_length, 0, 0)) * cq.Location(
        cq.Vector(0, 0, 0), cq.Vector(0, -1, 0), math.degrees(bucket_pitch(boom, bucket)))
    return {"front": front, "rear": rear, "boom": boom_l, "bucket": bucket_l}


def bucket_pitch(boom: float, bucket: float) -> float:
    """Bucket pitch relative to the boom: -boom keeps the bucket level (parallel lift), and the bucket
    state is the curl angle (planner: carry +0.55 curled back, dump -0.7 tipped)."""
    return -boom + bucket


# --------------------------------------------------------------------------- checks
def check(parts: list[Part]) -> list[str]:
    """Position and interference checks; returns the problems found (empty = pass)."""
    problems: list[str] = []
    tf = group_transforms(0, 0, 0)
    # 1. scanner optics centres where the twin ray-casts from
    want = {m.name: (m.x, m.y) for m in FRONT_MOUNTS}
    for m in REAR_MOUNTS:
        want[m.name] = (REAR_AXLE_X - m.x, -m.y)
    for pt in parts:
        if pt.name.startswith("scanner_"):
            c = pt.shape.val().moved(tf[pt.group]).Center()
            wx, wy = want[pt.name.removeprefix("scanner_")]
            # the housing centre sits 0.02 m behind the optics on the boresight; allow 0.04 m
            if math.hypot(c.x - wx, c.y - wy) > 0.04 or abs(c.z - SCAN_Z) > 0.03:
                problems.append(f"{pt.name} at ({c.x:.2f},{c.y:.2f},{c.z:.2f}), twin has ({wx:.2f},{wy:.2f},{SCAN_Z})")
    # 2. moving groups against kit parts over the joint ranges
    poses = [(g, b, k) for g in (-C.max_articulation, 0.0, C.max_articulation)
             for b in (C.boom_min, -0.2, C.boom_max) for k in (C.bucket_min, 0.0, C.bucket_max)]
    kit = [pt for pt in parts if pt.kit]
    movers = {"rear": [pt for pt in parts if pt.group == "rear" and not pt.kit and pt.name in
                       ("rear_frame", "cab_frame", "counterweight", "rear_tyre_left", "rear_tyre_right")],
              "boom": [pt for pt in parts if pt.group == "boom"],
              "bucket": [pt for pt in parts if pt.group == "bucket"]}
    seen = set()
    for g, b, k in poses:
        tf = group_transforms(g, b, k)
        for kp in kit:
            ks = kp.shape.val().moved(tf[kp.group])
            for grp, ms in movers.items():
                if grp == kp.group:
                    continue
                for mp in ms:
                    ms_ = mp.shape.val().moved(tf[mp.group])
                    a, bb = ks.BoundingBox(), ms_.BoundingBox()
                    if (a.xmax < bb.xmin or bb.xmax < a.xmin or a.ymax < bb.ymin or bb.ymax < a.ymin
                            or a.zmax < bb.zmin or bb.zmax < a.zmin):
                        continue
                    vol = ks.intersect(ms_).Volume()
                    if vol > 1e-6 and (kp.name, mp.name) not in seen:
                        seen.add((kp.name, mp.name))
                        problems.append(f"{kp.name} hits {mp.name} at articulation {math.degrees(g):.0f} deg, "
                                        f"boom {b:.2f}, bucket {k:.2f} ({vol * 1e6:.0f} cm3)")
    return problems


# --------------------------------------------------------------------------- export
def to_trimesh(shape: cq.Workplane, tol: float):
    import numpy as np
    import trimesh
    verts, faces = [], []
    for solid in shape.vals():
        vs, fs = solid.tessellate(tol, 0.3)
        off = len(verts)
        verts += [(v.x, v.y, v.z) for v in vs]
        faces += [(a + off, b + off, c + off) for a, b, c in fs]
    mesh = trimesh.Trimesh(np.array(verts), np.array(faces), process=True)
    return mesh.smooth_shaded if hasattr(mesh, "smooth_shaded") else mesh


def scanner_frames() -> list[dict]:
    """Scanner origin and boresight in each group's local frame (for the viewer's field fans)."""
    out = [{"name": m.name, "group": "front", "x": m.x, "y": m.y, "z": SCAN_Z, "yaw": m.heading} for m in FRONT_MOUNTS]
    ra = REAR_AXLE_X - JOINT_X
    out += [{"name": m.name, "group": "rear", "x": ra - m.x, "y": -m.y, "z": SCAN_Z, "yaw": math.pi - m.heading}
            for m in REAR_MOUNTS]
    return out


def export(parts: list[Part], problems: list[str] | None = None) -> dict:
    import numpy as np
    import trimesh
    from trimesh.visual.material import PBRMaterial
    from trimesh.visual.texture import TextureVisuals
    (OUT / "step").mkdir(parents=True, exist_ok=True)
    (OUT / "stl").mkdir(parents=True, exist_ok=True)
    for pt in parts:
        if pt.kit:
            cq.exporters.export(pt.shape, str(OUT / "step" / f"{pt.name}.step"))
            cq.exporters.export(pt.shape, str(OUT / "stl" / f"{pt.name}.stl"), tolerance=0.002, angularTolerance=0.2)
    tf = group_transforms()
    assy = cq.Assembly(name="kauha_loader")
    for pt in parts:
        assy.add(pt.shape, name=pt.name, loc=tf[pt.group], color=cq.Color(pt.color))
    assy.export(str(OUT / "kauha_assembly.step"))

    # GLB: y-up for three.js. Root node rotates z-up -> y-up; joint nodes carry the pivots, so the
    # viewer animates articulation, boom and bucket by rotating rear_joint / boom_joint / bucket_joint.
    scene = trimesh.Scene()
    zup_to_yup = trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0])
    scene.graph.update(frame_to="kauha", frame_from=scene.graph.base_frame, matrix=zup_to_yup)
    T = trimesh.transformations.translation_matrix
    scene.graph.update(frame_to="front_joint", frame_from="kauha", matrix=np.eye(4))
    scene.graph.update(frame_to="rear_joint", frame_from="front_joint", matrix=T([JOINT_X, 0, 0]))
    scene.graph.update(frame_to="boom_joint", frame_from="front_joint",
                       matrix=T([C.boom_pivot_forward, 0, C.boom_pivot_height]))
    scene.graph.update(frame_to="bucket_joint", frame_from="boom_joint", matrix=T([C.boom_length, 0, 0]))
    for pt in parts:
        mesh = to_trimesh(pt.shape, 0.01 if not pt.kit else 0.003)
        rgb = [int(pt.color[i:i + 2], 16) / 255 for i in (1, 3, 5)]
        mat = PBRMaterial(name=pt.name, baseColorFactor=rgb + [0.55 if "glass" in pt.name else 1.0],
                          metallicFactor=0.15 if pt.channel == "machine" else 0.3, roughnessFactor=0.6,
                          alphaMode="BLEND" if "glass" in pt.name else "OPAQUE", doubleSided=True)
        mesh.visual = TextureVisuals(material=mat)
        scene.add_geometry(mesh, geom_name=pt.name, node_name=pt.name, parent_node_name=f"{pt.group}_joint")
    glb = scene.export(file_type="glb", include_normals=True)
    (OUT / "kauha_kit.glb").write_bytes(glb)
    meta = {"frames": "x forward, y left, z up; origin front axle centre on the ground",
            "joints": {"rear_joint": {"at": [JOINT_X, 0, 0], "axis": [0, 0, -1], "range": [-C.max_articulation,
                                                                                           C.max_articulation]},
                       "boom_joint": {"at": [C.boom_pivot_forward, 0, C.boom_pivot_height], "axis": [0, -1, 0],
                                      "range": [C.boom_min, C.boom_max]},
                       "bucket_joint": {"at_boom": [C.boom_length, 0, 0], "axis": [0, -1, 0],
                                        "range": [C.bucket_min, C.bucket_max],
                                        "pitch": "-boom + bucket"}},
            "parts": [{"name": pt.name, "group": pt.group, "channel": pt.channel, "kit": pt.kit, "bom": pt.bom,
                       "note": pt.note, "color": pt.color} for pt in parts],
            "scanners": scanner_frames(),
            "checks": {"poses": 27, "kit_parts": sum(pt.kit for pt in parts), "problems": problems or [],
                       "layout_findings": LAYOUT_FINDINGS}}
    (OUT / "parts.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False))
    return meta


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="run the position and interference checks only")
    a = ap.parse_args()
    parts = build_parts()
    problems = check(parts)
    for pr in problems:
        print("FAIL", pr)
    print(f"{len(parts)} parts ({sum(p.kit for p in parts)} kit), checks: "
          f"{'pass' if not problems else str(len(problems)) + ' problems'}")
    if a.check:
        raise SystemExit(1 if problems else 0)
    if problems:
        raise SystemExit("not exporting a layout with interferences")
    export(parts, problems)
    print("exported to", OUT)


if __name__ == "__main__":
    main()
