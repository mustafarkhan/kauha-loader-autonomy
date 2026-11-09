"""KAUHA 1:8 testbed: parametric CAD, manufacturing drawings and BOM.

Everything is built in millimetres (STEP files open at the right size in
SolidWorks, Fusion 360, FreeCAD, Onshape). Frames: x forward from the front
axle, y left, z up from the ground; articulation joint at x = -200, rear axle at
x = -400 (1:8 of the reference machine, kauha.scale.profile).

Fabricated parts carry their 2D definition, so the same numbers make the solid,
the laser-cutting DXF and the drawing:
  * plates: 5 mm aluminium 5083 (frames), 6 mm (boom arms), 3 mm (links, lugs)
  * bucket: 1.5 mm steel shell (one bent sheet, flat pattern with bend lines)
    plus 3 mm side plates and cutting edge
  * turned parts: pins and spacers (lathe drawings)
  * printed parts: enclosures and brackets (STL, PETG)
Bought parts (hub-motor wheels, actuators, scanners, electronics) are envelopes
with their interface holes, for clearance checks only.

Actuator anchor points come from kauha/scale/linkage.py (hardware/scale/generated/
linkage.json), the same file the gateway firmware's length tables come from.

    python -m hardware.scale.cad.scale_cad            # build, check, export everything
    python -m hardware.scale.cad.scale_cad --check    # checks only
"""
from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

import cadquery as cq

from kauha.config import REPO_ROOT

LINK = json.loads((REPO_ROOT / "hardware" / "scale" / "generated" / "linkage.json").read_text())
OUT = Path(__file__).resolve().parent / "export"

MM = 1000.0
JOINT_X, REAR_AXLE_X = -200.0, -400.0
R_W, W_W = 82.5, 55.0                  # 6.5 in hub-motor wheel: radius, tyre width
TRACK = 262.5                          # wheel centre spacing
WY = TRACK / 2
AXLE_Z = R_W
PLATE_Y = 57.5                         # frame side plates, centre plane (5 mm: 55..60)
ARM_Y = 27.0                           # boom arms, centre plane (6 mm: 24..30)
LIFT_Y = 45.0                          # lift actuators
STEER_Z = 232.0                        # steering actuators run above the tyres and the hinge lugs
P = tuple(v * MM for v in LINK["boom_pivot"])
LB = LINK["boom_length"] * MM
ACT = LINK["actuators"]
TL = {k: (v * MM if isinstance(v, float) and k not in ("phi",) else v) for k, v in LINK["tilt_linkage"].items()}
REAR_OSC = math.radians(8)             # rear axle oscillation, each way
PIN_D = 6.35                           # PA-14P clevis hole
DENSITY = {"Al5083": 2.66e-6, "S235": 7.85e-6, "PETG": 1.27e-6, "bought": None}   # kg/mm^3

COLORS = {"frame": "#e2a312", "boom": "#e2a312", "bucket": "#3a3326", "link": "#8c8a83", "wheel": "#1f2122",
          "actuator": "#b8bcc2", "safety": "#d43b2a", "autonomy": "#2563c9", "ballast": "#5d636b",
          "enclosure": "#2b3036", "pin": "#d9dcdf"}


# ------------------------------------------------------------------ part model
@dataclass
class Plate2D:
    """A flat part: outline polygon and round holes, in the plate's own (u, v) frame (mm)."""
    outline: list
    holes: list = field(default_factory=list)       # (u, v, diameter)
    thickness: float = 5.0
    material: str = "Al5083"


@dataclass
class Part:
    name: str
    group: str                       # front | rear | rear_axle | boom | bucket | tiltlink | lift_L ... (moving body)
    shape: cq.Workplane
    kind: str                        # plate | sheet | turned | printed | bought
    material: str
    qty: int = 1
    color: str = "frame"
    bom: str = ""                    # supplier part or description for bought parts
    plate: Plate2D | None = None
    note: str = ""
    mass_kg: float | None = None     # bought parts: catalogue mass


def plane_xz(y_center: float, t: float) -> cq.Plane:
    """(u, v) -> (x, z) at y = y_center, thickness t centred on it."""
    return cq.Plane(origin=(0, y_center + t / 2, 0), xDir=(1, 0, 0), normal=(0, -1, 0))


def plane_xy(z_bottom: float) -> cq.Plane:
    return cq.Plane(origin=(0, 0, z_bottom), xDir=(1, 0, 0), normal=(0, 0, 1))


def plate_solid(p: Plate2D, plane: cq.Plane) -> cq.Workplane:
    s = cq.Workplane(plane).polyline(p.outline).close().extrude(p.thickness)
    for u, v, d in p.holes:
        s = s.cut(cq.Workplane(plane).center(u, v).circle(d / 2).extrude(p.thickness))
    return s


def box(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(axis: str, r: float, a0: float, a1: float, c1: float, c2: float) -> cq.Workplane:
    """Cylinder along an axis from a0 to a1; (c1, c2) are the other two coordinates in xyz order."""
    length = a1 - a0
    if axis == "y":
        return cq.Workplane("XZ").circle(r).extrude(-length).translate((c1, a0, c2))
    if axis == "x":
        return cq.Workplane("YZ").circle(r).extrude(length).translate((a0, c1, c2))
    return cq.Workplane("XY").circle(r).extrude(length).translate((c1, c2, a0))


def rod_between(p0, p1, r) -> cq.Workplane:
    v = cq.Vector(*p1) - cq.Vector(*p0)
    s = cq.Solid.makeCylinder(r, v.Length, cq.Vector(*p0), v.normalized())
    return cq.Workplane("XY").add(s)


# ------------------------------------------------------------------ kinematics (same as kauha/scale/linkage.py)
def boom_point(beta: float, u: float, w: float) -> tuple[float, float]:
    c, s = math.cos(beta), math.sin(beta)
    return P[0] + u * c - w * s, P[1] + u * s + w * c


def tilt_points(psi: float) -> dict:
    """Boom-frame points of the bucket linkage at bucket angle psi relative to the boom (mm)."""
    bu = LB
    du, dw = bu + TL["a"] * math.cos(psi + TL["phi"]), TL["a"] * math.sin(psi + TL["phi"])
    R = (bu - TL["g"], TL["h"])
    dx, dz = du - R[0], dw - R[1]
    d = math.hypot(dx, dz)
    b, c = TL["b"], TL["c"]
    a = (b * b - c * c + d * d) / (2 * d)
    h = math.sqrt(max(0.0, b * b - a * a))
    mx, mz = R[0] + a * dx / d, R[1] + a * dz / d
    E = (mx + TL["branch"] * h * (-dz) / d, mz + TL["branch"] * h * dx / d)
    return {"B": (bu, 0.0), "D": (du, dw), "R": R, "E": E, "F": (TL["fu"], TL["fw"])}


# ------------------------------------------------------------------ parts
def front_frame() -> list[Part]:
    lift_base = tuple(v * MM for v in ACT["lift"]["base"])
    side = Plate2D(
        outline=[(-150, 60), (30, 60), (70, 100), (135, 165), (135, 282), (118, 300), (92, 300), (40, 205),
                 (-150, 205)],
        holes=[(0, AXLE_Z, 14.0), (P[0], P[1], 10.2), (lift_base[0], lift_base[1], PIN_D + 0.15),
               (-135, 80, 5.5), (-135, 185, 5.5), (-60, 80, 5.5), (-60, 185, 5.5), (40, 80, 5.5)],
        thickness=5.0)
    parts = []
    for sgn, tag in ((1, "L"), (-1, "R")):
        parts.append(Part(f"FF_side_plate_{tag}", "front", plate_solid(side, plane_xz(sgn * PLATE_Y, 5.0)), "plate",
                          "Al5083", plate=side, note="front frame side plate, laser cut"))
    # hinge lugs (horizontal plates over/under the side plates) with the vertical joint pin hole
    lug = Plate2D(outline=[(-224, -24), (-190, -45), (-120, -60), (-120, 60), (-190, 45), (-224, 24)],
                  holes=[(JOINT_X, 0, 10.2), (-130, 50, 5.5), (-130, -50, 5.5), (-140, 0, 5.5)], thickness=5.0)
    parts.append(Part("FF_hinge_lug_top", "front", plate_solid(lug, plane_xy(205.0)), "plate", "Al5083", plate=lug,
                      note="bolts to the side plates through angle brackets"))
    parts.append(Part("FF_hinge_lug_bottom", "front", plate_solid(lug, plane_xy(55.0)), "plate", "Al5083",
                      plate=lug))
    # cross members: 20x20 aluminium profile between the side plates
    for i, (x, z) in enumerate(((-135, 80), (-135, 185), (-60, 80), (-60, 185), (40, 80))):
        parts.append(Part(f"FF_cross_{i}", "front", box(x - 10, x + 10, -55, 55, z - 10, z + 10), "bought", "Al6063",
                          color="link", bom="20x20 B-type slot 6 aluminium profile, cut to 110 mm, M5 tapped ends",
                          mass_kg=0.054))
    parts.append(Part("FF_boom_pivot_shaft", "front", cyl("y", 5.0, -62, 62, P[0], P[1]), "turned", "S235",
                      color="pin", note="10 h8 shaft through both side plates and both boom arms, with spacers"))
    # hub-motor axle clamp blocks (outside the side plates)
    for sgn, tag in ((1, "L"), (-1, "R")):
        y0, y1 = (60, 100) if sgn > 0 else (-100, -60)
        parts.append(Part(f"FF_axle_clamp_{tag}", "front", box(-20, 20, y0, y1, AXLE_Z - 20, AXLE_Z + 20)
                          .cut(cyl("y", 6.1, y0, y1, 0, AXLE_Z)), "printed", "Al6082", color="link",
                          note="split clamp for the hub-motor axle; machined aluminium (or printed for bench tests)"))
    # steering actuator front brackets (on top of the side plates)
    fx, fy = (v * MM for v in ACT["steer"]["base"])
    for sgn, tag in ((1, "L"), (-1, "R")):
        x = JOINT_X + fx
        parts.append(Part(f"FF_steer_bracket_{tag}", "front",
                          box(x - 15, x + 15, sgn * 55 if sgn > 0 else -fy - 8, fy + 8 if sgn > 0 else -55, 205,
                              STEER_Z - 14), "printed", "PETG", color="link",
                          note="steering actuator clevis bracket; PETG for bench, aluminium for field tests"))
    # front scanner (safety) and autonomy mast
    parts.append(Part("FF_front_scanner", "front", box(40, 157.5, -53.3, 53.3, 35, 115), "bought", "-", color="safety",
                      bom="SICK nanoScan3 Pro I/O NANS3-CAAZ30AN1 (or 2D lidar for the evaluator profile)",
                      mass_kg=0.67, note="scan plane about 85 mm above ground"))
    parts.append(Part("FF_mast", "front", box(-135, -115, -10, 10, 205, 430), "bought", "Al6063", color="autonomy",
                      bom="20x20 profile, 225 mm", mass_kg=0.11))
    parts.append(Part("FF_mast_sensors", "front", cyl("z", 38, 430, 490, -125, 0), "bought", "-", color="autonomy",
                      bom="autonomy lidar + GNSS antenna (envelope)", mass_kg=0.4))
    return parts


def rear_frame() -> list[Part]:
    rx, ry = (v * MM for v in ACT["steer"]["rod"])
    side = Plate2D(outline=[(-262, 65), (-350, 65), (-350, 122), (-450, 122), (-450, 65), (-612.5, 65),
                            (-612.5, 195), (-262, 195)],
                   holes=[(-275, 85, 5.5), (-275, 175, 5.5), (-370, 85, 5.5), (-430, 85, 5.5), (-520, 85, 5.5),
                          (-520, 175, 5.5), (-600, 85, 5.5), (-600, 175, 5.5)], thickness=5.0)
    parts = []
    for sgn, tag in ((1, "L"), (-1, "R")):
        parts.append(Part(f"RF_side_plate_{tag}", "rear", plate_solid(side, plane_xz(sgn * PLATE_Y, 5.0)), "plate",
                          "Al5083", plate=side))
    deck = Plate2D(outline=[(-262, -60), (-612.5, -60), (-612.5, 60), (-262, 60)],
                   holes=[(-275, 50, 5.5), (-275, -50, 5.5), (-600, 50, 5.5), (-600, -50, 5.5), (-420, 0, 25)],
                   thickness=4.0)
    parts.append(Part("RF_deck", "rear", plate_solid(deck, plane_xy(195.0)), "plate", "Al5083", plate=deck))
    # hinge block between the front lugs (vertical joint pin)
    block = box(-262, -182, -30, 30, 61, 204).cut(cyl("z", 5.1, 61, 204, JOINT_X, 0))
    parts.append(Part("RF_hinge_block", "rear", block, "turned", "Al6082", color="link",
                      note="machined block, flanged bronze bushes top and bottom"))
    parts.append(Part("RF_front_cross", "rear", box(-272, -262, -60, 60, 65, 195), "plate", "Al5083",
                      plate=Plate2D([(-60, 65), (60, 65), (60, 195), (-60, 195)], [(-50, 85, 5.5), (50, 85, 5.5),
                                                                                   (0, 130, 5.5)], 10.0)))
    # trunnion plates for the oscillating rear axle
    for x, tag in ((-365, "F"), (-435, "R")):
        tp = Plate2D([(-55, 40), (55, 40), (55, 195), (-55, 195)], [(0, AXLE_Z, 10.2)], 5.0)
        parts.append(Part(f"RF_trunnion_{tag}", "rear", box(x - 2.5, x + 2.5, -55, 55, 40, 195)
                          .cut(cyl("x", 5.1, x - 3, x + 3, 0, AXLE_Z)), "plate", "Al5083", plate=tp))
    for sgn, tag in ((1, "L"), (-1, "R")):
        x = JOINT_X + rx
        parts.append(Part(f"RF_steer_bracket_{tag}", "rear",
                          box(x - 15, x + 15, sgn * 55 if sgn > 0 else -ry - 8, ry + 8 if sgn > 0 else -55, 199,
                              STEER_Z - 14), "printed", "PETG", color="link"))
    parts.append(Part("RF_counterweight", "rear", box(-655, -545, -60, 60, 199, 309), "turned", "S235",
                      color="ballast", note="steel ballast block; trim length to set the static tipping load"))
    parts.append(Part("RF_battery", "rear", box(-560, -460, -50, 50, 70, 185), "bought", "-", color="enclosure",
                      bom="36 V 10S Li-ion pack, 10 Ah, with BMS and XT60 (e-bike type)", mass_kg=2.4))
    parts.append(Part("RF_cab_electronics", "rear", box(-440, -300, -44, 44, 199, 330), "printed", "PETG",
                      color="enclosure", mass_kg=1.4,
                      note="cab: PLC (Pi 4 + CAN HAT + 24 V I/O), gateway, sensor node, relays; printed shell"))
    parts.append(Part("RF_cab_roof", "rear", box(-448, -292, -50, 50, 330, 336), "printed", "PETG", color="enclosure"))
    for sgn, tag in ((1, "L"), (-1, "R")):
        parts.append(Part(f"RF_estop_{tag}", "rear", cyl("y", 20, 44 if sgn > 0 else -64, 64 if sgn > 0 else -44,
                                                           -362, 300), "bought", "-", color="safety",
                          bom="E-stop pushbutton 22 mm, 2 NC, with yellow enclosure", mass_kg=0.08))
    parts.append(Part("RF_rear_scanner", "rear", box(-612.5, -495, -53.3, 53.3, 35, 115)
                      .translate((-117.5, 0, 0)), "bought", "-", color="safety",
                      bom="SICK nanoScan3 Pro I/O NANS3-CAAZ30AN1 (or 2D lidar for the evaluator profile)",
                      mass_kg=0.67))
    return parts


def rear_axle() -> list[Part]:
    beam = box(REAR_AXLE_X - 20, REAR_AXLE_X + 20, -100, 100, AXLE_Z - 20, AXLE_Z + 20)
    beam = beam.cut(box(REAR_AXLE_X - 17, REAR_AXLE_X + 17, -103, 103, AXLE_Z - 17, AXLE_Z + 17))
    beam = beam.cut(cyl("x", 5.1, REAR_AXLE_X - 21, REAR_AXLE_X + 21, 0, AXLE_Z))
    parts = [Part("RA_beam", "rear_axle", beam, "turned", "Al6063", color="link",
                  note="40x40x3 aluminium square tube, 200 mm; 10 mm cross bore for the oscillation pin")]
    for sgn, tag in ((1, "L"), (-1, "R")):
        y0, y1 = (100, 104) if sgn > 0 else (-104, -100)
        parts.append(Part(f"RA_axle_clamp_{tag}", "rear_axle", box(REAR_AXLE_X - 20, REAR_AXLE_X + 20, y0, y1,
                                                                    AXLE_Z - 20, AXLE_Z + 20), "printed", "Al6082",
                          color="link"))
    return parts


def wheels() -> list[Part]:
    parts = []
    for grp, x in (("front", 0.0), ("rear_axle", REAR_AXLE_X)):
        for sgn, tag in ((1, "L"), (-1, "R")):
            y0 = sgn * (WY - W_W / 2) if sgn > 0 else -(WY + W_W / 2)
            w = cyl("y", R_W, y0, y0 + W_W, x, AXLE_Z)
            parts.append(Part(f"{'FW' if grp == 'front' else 'RW'}_hub_motor_{tag}", grp, w, "bought", "-",
                              color="wheel", bom="6.5 in hoverboard hub motor, 36 V 350 W, Hall sensors, solid tyre",
                              mass_kg=3.0))
    return parts


def boom() -> list[Part]:
    """Boom group: drawn in its own frame at boom angle 0 (pivot at P, u along the boom)."""
    lift_rod = tuple(v * MM for v in ACT["lift"]["rod"])
    F, R = (TL["fu"], TL["fw"]), (LB - TL["g"], TL["h"])
    arm = Plate2D(outline=[(-18, -15), (LB + 18, -15), (LB + 18, 15), (lift_rod[0] + 14, 15),
                           (lift_rod[0] + 14, 15), (-18, 15)],
                  holes=[(0, 0, 10.2), (LB, 0, 10.2), (F[0], F[1], 8.2), (R[0], R[1], 8.2), (150, 0, 8.2)],
                  thickness=6.0)
    # lift actuator lug below the arm
    arm.outline = [(-18, -15), (45, -15), (50, -42), (75, -42), (80, -15), (lift_rod[0] - 16, -15),
                   (lift_rod[0] - 12, lift_rod[1] - 12), (lift_rod[0] + 12, lift_rod[1] - 12),
                   (lift_rod[0] + 16, -15), (LB + 12, -15), (LB + 12, 15), (R[0] + 16, 15), (R[0] + 12, R[1] + 12),
                   (R[0] - 12, R[1] + 12), (R[0] - 16, 15), (-18, 15)]
    arm.holes = [h if h[:2] != (150, 0) else (62, -28, 20.2) for h in arm.holes]
    arm.holes.append((lift_rod[0], lift_rod[1], PIN_D + 0.15))
    parts = []
    for sgn, tag in ((1, "L"), (-1, "R")):
        s = plate_solid(arm, plane_xz(sgn * ARM_Y, 6.0)).translate((P[0], 0, P[1]))
        parts.append(Part(f"BM_arm_{tag}", "boom", s, "plate", "Al5083", color="boom", plate=arm,
                          note="drawn at boom angle 0; holes: pivot, bucket pivot, tilt base, side-link pivot, tube"))
    for name, (u, w), d in (("BM_cross_tube", (62, -28), 20), ("BM_pin_tilt_base", F, 8), ("BM_pin_side_link", R, 8)):
        parts.append(Part(name, "boom", cyl("y", d / 2, -ARM_Y - 3, ARM_Y + 3, P[0] + u, P[1] + w), "turned",
                          "S235" if d < 20 else "Al6063", color="pin"))
    for sgn, tag in ((1, "L"), (-1, "R")):
        parts.append(Part(f"BM_pin_lift_{tag}", "boom",
                          cyl("y", PIN_D / 2, min(sgn * 24, sgn * 52), max(sgn * 24, sgn * 52),
                              P[0] + lift_rod[0], P[1] + lift_rod[1]), "turned", "S235", color="pin"))
    return parts


# bucket profile in the bucket frame (pivot at origin, x along the floor at angle 0, z up), mm
BUCKET_PROFILE = [(160.0, -30.0), (34.0, -26.0), (22.0, -8.0), (22.0, 122.0), (74.0, 144.0)]
BUCKET_W = 337.5
BUCKET_T = 1.5


def bucket() -> list[Part]:
    prof = BUCKET_PROFILE
    side = Plate2D(outline=prof, holes=[], thickness=3.0, material="S235")
    parts = []
    for sgn, tag in ((1, "L"), (-1, "R")):
        yc = sgn * (BUCKET_W / 2 - 1.5)
        parts.append(Part(f"BK_side_{tag}", "bucket", plate_solid(side, plane_xz(yc, 3.0)), "plate", "S235",
                          color="bucket", plate=side))
    # shell: lip -> heel -> back -> top, as thin strips (bent from one sheet)
    shell = None
    for (x0, z0), (x1, z1) in zip(prof[:-1], prof[1:], strict=True):
        seg = rod_between((x0, -BUCKET_W / 2 + 3, z0), (x1, -BUCKET_W / 2 + 3, z1), 0.1)  # placeholder for direction
        L = math.hypot(x1 - x0, z1 - z0)
        ang = math.degrees(math.atan2(z1 - z0, x1 - x0))
        strip = (cq.Workplane("XY").box(L, BUCKET_W - 6, BUCKET_T).translate((L / 2, 0, -BUCKET_T / 2))
                 .rotate((0, 0, 0), (0, 1, 0), -ang).translate((x0, 0, z0)))
        shell = strip if shell is None else shell.union(strip)
        del seg
    parts.append(Part("BK_shell", "bucket", shell, "sheet", "S235", color="bucket",
                      note="one 1.5 mm sheet, four bends; flat pattern in the DXF"))
    edge = rod_between((160, -BUCKET_W / 2, -30), (160, BUCKET_W / 2, -30), 3.0)
    parts.append(Part("BK_cutting_edge", "bucket", edge, "turned", "S235", color="pin",
                      note="6 mm round bar welded along the lip (cutting edge)"))
    # pivot lugs (outside the boom arms) and H-link lugs (centre)
    D = (TL["a"] * math.cos(TL["phi"]), TL["a"] * math.sin(TL["phi"]))
    lug = Plate2D(outline=[(-14, -16), (16, -24), (16, 30), (-14, 14)], holes=[(0, 0, 10.2)], thickness=4.0,
                  material="S235")
    for sgn, tag in ((1, "L"), (-1, "R")):
        parts.append(Part(f"BK_pivot_lug_{tag}", "bucket", plate_solid(lug, plane_xz(sgn * 33.0, 4.0)), "plate",
                          "S235", color="bucket", plate=lug))
    hl = Plate2D(outline=[(D[0] - 12, D[1] - 12), (16, D[1] - 30), (16, D[1] + 14), (D[0], D[1] + 12),
                          (D[0] - 12, D[1])], holes=[(D[0], D[1], 8.2)], thickness=3.0, material="S235")
    for sgn, tag in ((1, "L"), (-1, "R")):
        parts.append(Part(f"BK_hlink_lug_{tag}", "bucket", plate_solid(hl, plane_xz(sgn * 16.0, 3.0)), "plate",
                          "S235", color="bucket", plate=hl))
    return parts


def tilt_links(psi: float) -> list[Part]:
    """Side links and H-links, placed for bucket angle psi; drawn in the boom frame."""
    pts = tilt_points(psi)
    parts = []

    def link_plate(a, b, name, yc):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        pl = Plate2D(outline=[(-10, -9), (L + 10, -9), (L + 10, 9), (-10, 9)], holes=[(0, 0, 8.2), (L, 0, 8.2)],
                     thickness=3.0, material="S235")
        ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        s = (plate_solid(pl, plane_xz(yc, 3.0)).rotate((0, 0, 0), (0, 1, 0), -ang)
             .translate((P[0] + a[0], 0, P[1] + a[1])))
        return Part(name, "tiltlink", s, "plate", "S235", color="link", plate=pl,
                    note=f"centre distance {L:.1f} mm")

    for sgn, tag in ((1, "L"), (-1, "R")):
        parts.append(link_plate(pts["R"], pts["E"], f"TL_side_link_{tag}", sgn * 7.5))
        parts.append(link_plate(pts["E"], pts["D"], f"TL_h_link_{tag}", sgn * 11.5))
    return parts


def actuators(pose: dict) -> list[Part]:
    """PA-14P envelopes between their pins at the given pose (world frame)."""
    out = []
    beta, psi, gam = pose["boom"], pose["bucket"] - pose["boom"], pose["gamma"]
    lb, lr = (v * MM for v in ACT["lift"]["base"]), (v * MM for v in ACT["lift"]["rod"])
    lb, lr = tuple(lb), tuple(lr)
    for sgn, tag in ((1, "L"), (-1, "R")):
        a = (lb[0], sgn * LIFT_Y, lb[1])
        cx, cz = boom_point(beta, *lr)
        out.append(_actuator(f"ACT_lift_{tag}", a, (cx, sgn * LIFT_Y, cz), ACT["lift"]))
    pts = tilt_points(psi)
    F = boom_point(beta, *pts["F"])
    E = boom_point(beta, *pts["E"])
    out.append(_actuator("ACT_tilt", (F[0], 0, F[1]), (E[0], 0, E[1]), ACT["tilt"]))
    fx, fy = (v * MM for v in ACT["steer"]["base"])
    rx, ry = (v * MM for v in ACT["steer"]["rod"])
    for sgn, tag in ((1, "L"), (-1, "R")):
        # rear-frame point turned about the joint by -gamma (front frame fixed)
        c, s = math.cos(-gam), math.sin(-gam)
        qx, qy = rx * c - sgn * ry * s, rx * s + sgn * ry * c
        out.append(_actuator(f"ACT_steer_{tag}", (JOINT_X + fx, sgn * fy, STEER_Z), (JOINT_X + qx, qy, STEER_Z),
                             ACT["steer"]))
    return out


def _actuator(name, p0, p1, spec) -> Part:
    """Body (25 x 40 mm section approximated as a 13 mm radius tube) from 25 mm after the base pin to the rod,
    rod Ø10 to the rod-end pin."""
    v = cq.Vector(*p1) - cq.Vector(*p0)
    L = v.Length
    u = v.normalized()
    stroke = spec["stroke_in"] * 25.4
    body_end = L - (L - (stroke + 140.0)) - 40.0       # body covers retracted length less the rod end
    body = rod_between(tuple(cq.Vector(*p0) + u * 22), tuple(cq.Vector(*p0) + u * max(60.0, body_end)), 12.5)
    rod = rod_between(tuple(cq.Vector(*p0) + u * max(60.0, body_end)), tuple(cq.Vector(*p1) - u * 8), 5.0)
    return Part(name, "world", body.union(rod), "bought", "-", color="actuator",
                bom=f"Progressive Automations PA-14P-{spec['stroke_in']:g}-{spec['force_lbf']} (12 V, pot feedback)",
                mass_kg=0.7, note=f"pin-to-pin {L:.1f} mm")


# ------------------------------------------------------------------ poses and assembly
CARRY = {"gamma": 0.0, "boom": -0.2, "bucket": 0.55, "osc": 0.0}


def transforms(pose: dict) -> dict:
    """cq.Location of each moving group (front frame fixed)."""
    gam, beta = pose["gamma"], pose["boom"]
    bucket_world = pose["bucket"]               # world pitch of the bucket (twin convention)
    rear = cq.Location(cq.Vector(JOINT_X, 0, 0), cq.Vector(0, 0, 1), -math.degrees(gam)) * \
        cq.Location(cq.Vector(-JOINT_X, 0, 0))
    osc = rear * cq.Location(cq.Vector(REAR_AXLE_X, 0, AXLE_Z), cq.Vector(1, 0, 0), math.degrees(pose["osc"])) * \
        cq.Location(cq.Vector(-REAR_AXLE_X, 0, -AXLE_Z))
    boom_loc = cq.Location(cq.Vector(P[0], 0, P[1]), cq.Vector(0, 1, 0), -math.degrees(beta)) * \
        cq.Location(cq.Vector(-P[0], 0, -P[1]))
    bx, bz = boom_point(beta, LB, 0.0)
    bucket_loc = cq.Location(cq.Vector(bx, 0, bz), cq.Vector(0, 1, 0), -math.degrees(bucket_world))
    return {"front": cq.Location(), "rear": rear, "rear_axle": osc, "boom": boom_loc, "tiltlink": boom_loc,
            "bucket": bucket_loc, "world": cq.Location()}


def build(pose: dict = CARRY) -> list[Part]:
    psi = pose["bucket"] - pose["boom"]
    return (front_frame() + rear_frame() + rear_axle() + wheels() + boom() + bucket() + tilt_links(psi)
            + actuators(pose))


def placed(parts: list[Part], pose: dict) -> list[tuple[Part, cq.Shape]]:
    tf = transforms(pose)
    out = []
    for p in parts:
        shp = p.shape.val() if len(p.shape.vals()) == 1 else cq.Compound.makeCompound(p.shape.vals())
        out.append((p, shp.moved(tf[p.group])))
    return out


# ------------------------------------------------------------------ checks
POSES = [dict(gamma=g, boom=b, bucket=k, osc=o)
         for g in (-0.69, 0.0, 0.69)
         for b, k in ((-0.62, 0.0), (-0.62, 0.30), (-0.2, 0.55), (0.4, 0.3), (0.85, -0.7), (0.95, 0.5))
         for o in ((-REAR_OSC, 0.0, REAR_OSC) if g == 0.0 else (0.0,))]
# pairs that touch by design (pinned or bolted together)
JOINED = [("FF_side_plate", "FF_hinge_lug"), ("FF_side_plate", "FF_cross"), ("FF_side_plate", "FF_axle_clamp"),
          ("FF_side_plate", "FF_steer_bracket"), ("FF_hinge_lug", "RF_hinge_block"), ("RF_side_plate", "RF_deck"),
          ("RF_side_plate", "RF_front_cross"), ("RF_hinge_block", "RF_front_cross"), ("RF_side_plate", "RF_trunnion"),
          ("RF_deck", "RF_trunnion"), ("RF_deck", "RF_steer_bracket"), ("RF_side_plate", "RF_steer_bracket"),
          ("RF_deck", "RF_counterweight"), ("RF_deck", "RF_cab"), ("RF_cab", "RF_estop"), ("RF_cab_electronics",
                                                                                         "RF_cab_roof"),
          ("RF_side_plate", "RF_battery"), ("RF_side_plate", "RF_rear_scanner"), ("RA_beam", "RA_axle_clamp"),
          ("RA_beam", "RF_trunnion"), ("RA_axle_clamp", "RW_hub_motor"), ("FF_axle_clamp", "FW_hub_motor"),
          ("BM_arm", "BM_"), ("BM_pin", "TL_"), ("BM_pin", "ACT_tilt"), ("BM_pin_lift", "ACT_lift"),
          ("TL_side_link", "TL_h_link"), ("TL_", "ACT_tilt"), ("TL_h_link", "BK_hlink_lug"), ("BK_", "BK_"),
          ("BM_arm", "BK_pivot_lug"), ("BM_arm", "ACT_lift"), ("FF_side_plate", "ACT_lift"),
          ("FF_steer_bracket", "ACT_steer"), ("RF_steer_bracket", "ACT_steer"), ("FF_side_plate", "BM_"),
          ("FF_mast", "FF_hinge_lug"), ("FF_mast", "FF_mast_sensors"), ("FF_cross", "FF_hinge_lug"),
          ("RF_front_cross", "RF_deck"), ("RF_trunnion", "RF_battery"), ("FF_cross", "FF_front_scanner"),
          ("TL_", "BK_pivot_lug"), ("BK_hlink_lug", "BM_pin")]


def _joined(a: str, b: str) -> bool:
    return any((a.startswith(x) and b.startswith(y)) or (a.startswith(y) and b.startswith(x)) for x, y in JOINED)


def interference(pose: dict, parts: list[Part] | None = None, tol_mm3: float = 5.0) -> list[str]:
    pl = placed(parts or build(pose), pose)
    boxes = [s.BoundingBox() for _, s in pl]
    out = []
    for i in range(len(pl)):
        for j in range(i + 1, len(pl)):
            a, b = pl[i][0], pl[j][0]
            if a.name == b.name or _joined(a.name, b.name):
                continue
            bi, bj = boxes[i], boxes[j]
            if bi.xmax < bj.xmin or bj.xmax < bi.xmin or bi.ymax < bj.ymin or bj.ymax < bi.ymin or \
                    bi.zmax < bj.zmin or bj.zmax < bi.zmin:
                continue
            v = pl[i][1].intersect(pl[j][1]).Volume()
            if v > tol_mm3:
                out.append(f"{a.name} x {b.name}: {v:.0f} mm3")
    return out


def ground_check(pose: dict) -> list[str]:
    """Nothing but tyres (and the bucket while digging) below 25 mm."""
    out = []
    for p, s in placed(build(pose), pose):
        if "hub_motor" in p.name or p.group == "bucket" or p.name.startswith(("TL_", "ACT_tilt")):
            continue
        z = s.BoundingBox().zmin
        if z < 25.0:
            out.append(f"{p.name} {z:.1f} mm above ground")
    return out


def actuator_length_check() -> list[str]:
    out = []
    for pose in POSES:
        for p in actuators(pose):
            spec = ACT[p.name.split("_")[1]]
            L = float(p.note.split()[1])
            lo, hi = (v * MM for v in spec["length_range_m"])
            if not lo - 0.5 <= L <= hi + 0.5:
                out.append(f"{p.name} {L:.1f} mm outside {lo:.0f}-{hi:.0f} at {pose}")
    return out


def mass_properties(parts: list[Part], pose: dict = CARRY) -> tuple[float, tuple, list]:
    rows, total, mom = [], 0.0, [0.0, 0.0, 0.0]
    for p, shp in placed(parts, pose):
        if p.mass_kg is not None:
            m = p.mass_kg * p.qty
        else:
            rho = DENSITY.get(p.material) or (DENSITY["Al5083"] if p.material.startswith("Al") else 7.85e-6)
            m = shp.Volume() * rho
        c = shp.Center()
        total += m
        mom = [mom[0] + m * c.x, mom[1] + m * c.y, mom[2] + m * c.z]
        rows.append((p.name, m))
    return total, tuple(v / total for v in mom), rows


def tipping(pose: dict | None = None) -> dict:
    """Static tipping load in the sense of ISO 14397-1: the load at the bucket that lifts the rear wheels.

    Articulated machine at full turn: the rear axle oscillates, so the rear support is
    its pivot on the centre line; the tipping line runs from the outer front tyre
    contact to that pivot. Straight ahead it is the front axle line.
    """
    pose = pose or dict(gamma=0.69, boom=-0.2, bucket=0.55, osc=0.0)
    parts = build(pose)
    tot, cg, _ = mass_properties(parts, pose)
    g = pose["gamma"]
    c, s = math.cos(-g), math.sin(-g)
    piv = (JOINT_X + (REAR_AXLE_X - JOINT_X) * c, (REAR_AXLE_X - JOINT_X) * s)
    bx, _ = boom_point(pose["boom"], LB, 0.0)
    load = (bx + 60.0, 0.0)                      # load centre ahead of the bucket pivot

    def side_dist(a, b, pt):
        return ((b[0] - a[0]) * (pt[1] - a[1]) - (b[1] - a[1]) * (pt[0] - a[0])) / math.hypot(b[0] - a[0],
                                                                                               b[1] - a[1])
    # support triangle; the tipping edge is the first one the combined CG crosses on its way to the load
    tri = [(0.0, WY), (0.0, -WY), piv]
    best = None
    for k in range(3):
        a, b = tri[k], tri[(k + 1) % 3]
        d_cg, d_load = side_dist(a, b, cg[:2]), side_dist(a, b, load)
        if d_cg * d_load < 0:
            f = d_cg / (d_cg - d_load)            # fraction of the way to the load where the edge is crossed
            if best is None or f < best[0]:
                best = (f, abs(d_cg), abs(d_load))
    loads = [tot * best[1] / best[2]] if best else []
    tip_turn = min(loads) if loads else float("inf")
    tot0, cg0, _ = mass_properties(build(dict(pose, gamma=0.0)), dict(pose, gamma=0.0))
    tip_straight = tot0 * abs(cg0[0]) / load[0] if cg0[0] < 0 else 0.0
    return {"mass_kg": round(tot, 2), "cg_mm": [round(v, 1) for v in cg],
            "tipping_load_straight_kg": round(tip_straight, 2), "tipping_load_full_turn_kg": round(tip_turn, 2),
            "rated_operating_load_kg": round(0.5 * tip_turn, 2)}


def bucket_volume_l() -> float:
    xs, zs = zip(*BUCKET_PROFILE, strict=True)
    area = 0.5 * abs(sum(xs[i] * zs[(i + 1) % len(xs)] - xs[(i + 1) % len(xs)] * zs[i] for i in range(len(xs))))
    return area * (BUCKET_W - 6) / 1e6


def check(parts: list[Part] | None = None) -> dict:
    parts = parts or build(CARRY)
    res = {"interference": {}, "ground": ground_check(CARRY), "actuators": actuator_length_check()}
    for pose in POSES:
        bad = interference(pose)
        if bad:
            res["interference"][json.dumps({k: round(v, 2) for k, v in pose.items()})] = bad
    res["stability"] = tipping()
    res["bucket_struck_volume_l"] = round(bucket_volume_l(), 2)
    return res


# ------------------------------------------------------------------ manufacturing outputs
TITLE = "KAUHA 1:8 testbed"
TOLERANCE = "General tolerances ISO 2768-m. Break sharp edges 0.3 mm. Units mm."
SHEET_K = 0.44                         # K-factor, 1.5 mm mild steel, air bending
BEND_R = 1.5                           # inner bend radius


def plate_groups(parts: list[Part]) -> list[tuple[str, Plate2D, list[str]]]:
    """Identical flat parts (mirrored left/right pairs) share one drawing."""
    groups: dict = {}
    for p in parts:
        if p.plate is None:
            continue
        key = (json.dumps([[round(a, 2), round(b, 2)] for a, b in p.plate.outline]),
               json.dumps([[round(c, 2) for c in h] for h in p.plate.holes]), p.plate.thickness, p.plate.material)
        groups.setdefault(key, (p.plate, []))[1].append(p.name)
    out = []
    for (_, _, _, _), (pl, names) in groups.items():
        base = os.path.commonprefix(names).rstrip("_") if len(names) > 1 else names[0]
        out.append((base, pl, names))
    return sorted(out)


def write_dxf(name: str, pl: Plate2D, path: Path) -> None:
    import ezdxf
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    doc.layers.add("CUT", color=7)
    doc.layers.add("INFO", color=3)
    msp = doc.modelspace()
    pts = [(float(u), float(v)) for u, v in pl.outline]
    msp.add_lwpolyline(pts, close=True, dxfattribs={"layer": "CUT"})
    for u, v, d in pl.holes:
        msp.add_circle((u, v), d / 2, dxfattribs={"layer": "CUT"})
    xs, ys = zip(*pts, strict=True)
    msp.add_text(f"{name}  {pl.material}  t={pl.thickness:g} mm", height=4,
                 dxfattribs={"layer": "INFO"}).set_placement((min(xs), min(ys) - 10))
    doc.saveas(path)


def shell_flat_pattern() -> dict:
    """Flat pattern of the bucket shell: segment lengths along the profile, bend allowances."""
    prof = BUCKET_PROFILE
    segs = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(prof[:-1], prof[1:], strict=True)]
    angles = []
    for i in range(1, len(prof) - 1):
        a1 = math.atan2(prof[i][1] - prof[i - 1][1], prof[i][0] - prof[i - 1][0])
        a2 = math.atan2(prof[i + 1][1] - prof[i][1], prof[i + 1][0] - prof[i][0])
        angles.append(abs((a2 - a1 + math.pi) % (2 * math.pi) - math.pi))
    # mold-line lengths minus bend deductions
    bd = [2 * (BEND_R + BUCKET_T) * math.tan(t / 2) - t * (BEND_R + SHEET_K * BUCKET_T) for t in angles]
    flat_segs = [s for s in segs]
    for i, d in enumerate(bd):
        flat_segs[i] -= d / 2
        flat_segs[i + 1] -= d / 2
    return {"segments_mm": flat_segs, "bend_angles_deg": [math.degrees(t) for t in angles],
            "flat_length_mm": sum(flat_segs), "width_mm": BUCKET_W - 6}


def write_shell_dxf(path: Path) -> dict:
    import ezdxf
    fp = shell_flat_pattern()
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    doc.layers.add("CUT", color=7)
    doc.layers.add("BEND", color=1, linetype="DASHED")
    doc.layers.add("INFO", color=3)
    msp = doc.modelspace()
    L, W = fp["flat_length_mm"], fp["width_mm"]
    msp.add_lwpolyline([(0, 0), (L, 0), (L, W), (0, W)], close=True, dxfattribs={"layer": "CUT"})
    x = 0.0
    for s, a in zip(fp["segments_mm"][:-1], fp["bend_angles_deg"], strict=True):
        x += s
        msp.add_line((x, 0), (x, W), dxfattribs={"layer": "BEND"})
        msp.add_text(f"BEND {a:.0f} deg R{BEND_R:g}", height=4, dxfattribs={"layer": "INFO", "rotation": 90}
                     ).set_placement((x - 2, 10))
    msp.add_text(f"BK_shell flat pattern  S235 t={BUCKET_T} mm  K={SHEET_K}", height=5,
                 dxfattribs={"layer": "INFO"}).set_placement((0, -12))
    doc.saveas(path)
    return fp


def _draw_plate(ax, name: str, pl: Plate2D, qty: int) -> None:
    import matplotlib.patches as mp
    pts = pl.outline + [pl.outline[0]]
    xs, ys = zip(*pts, strict=True)
    ax.plot(xs, ys, color="#111", lw=1.0)
    for i, (u, v, d) in enumerate(pl.holes):
        ax.add_patch(mp.Circle((u, v), d / 2, fill=False, lw=0.8, color="#111"))
        ax.annotate(f"H{i + 1}", (u, v), xytext=(4, 4), textcoords="offset points", fontsize=6, color="#2563c9")
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    span = max(x1 - x0, y1 - y0)
    off = 0.06 * span
    ax.annotate("", (x0, y0 - off), (x1, y0 - off), arrowprops=dict(arrowstyle="<->", lw=0.6))
    ax.text((x0 + x1) / 2, y0 - off * 1.25, f"{x1 - x0:.1f}", ha="center", va="top", fontsize=7)
    ax.annotate("", (x0 - off, y0), (x0 - off, y1), arrowprops=dict(arrowstyle="<->", lw=0.6))
    ax.text(x0 - off * 1.25, (y0 + y1) / 2, f"{y1 - y0:.1f}", ha="right", va="center", rotation=90, fontsize=7)
    ax.set_aspect("equal")
    ax.set_xlim(x0 - 0.25 * span, x1 + 0.1 * span)
    ax.set_ylim(y0 - 0.25 * span, y1 + 0.1 * span)
    ax.axis("off")
    ax.set_title(f"{name}   (datum: lower-left = ({x0:.1f}, {y0:.1f}) in the part frame)", fontsize=8, loc="left")


def _title_block(fig, name: str, material: str, t: float | None, qty: int, process: str, extra: str = "") -> None:
    fig.text(0.04, 0.035, f"{TITLE}  |  {name}  |  {material}" + (f"  t={t:g} mm" if t else "") +
             f"  |  qty {qty}  |  {process}", fontsize=8, family="monospace")
    fig.text(0.04, 0.015, TOLERANCE + ("  " + extra if extra else ""), fontsize=7, family="monospace",
             color="#444")


def write_drawings(parts: list[Part], path: Path, renders: list[Path]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    with PdfPages(path) as pdf:
        fig = plt.figure(figsize=(11.69, 8.27))
        fig.suptitle(f"{TITLE}: general arrangement", x=0.04, ha="left", fontsize=13)
        for i, r in enumerate(renders[:4]):
            ax = fig.add_axes([0.03 + 0.48 * (i % 2), 0.48 - 0.42 * (i // 2), 0.46, 0.42])
            ax.imshow(mpimg.imread(r))
            ax.axis("off")
        _title_block(fig, "assembly", "see BOM", None, 1, "STEP: kauha_scale8_assembly.step")
        pdf.savefig(fig)
        plt.close(fig)
        for base, pl, names in plate_groups(parts):
            fig = plt.figure(figsize=(11.69, 8.27))
            ax = fig.add_axes([0.04, 0.1, 0.62, 0.82])
            _draw_plate(ax, base, pl, len(names))
            tab = fig.add_axes([0.70, 0.1, 0.27, 0.82])
            tab.axis("off")
            rows = [["hole", "u", "v", "dia"]] + [[f"H{i + 1}", f"{u:.1f}", f"{v:.1f}", f"{d:.2f}"]
                                                  for i, (u, v, d) in enumerate(pl.holes)]
            if len(rows) > 1:
                t = tab.table(cellText=rows, loc="upper left", colWidths=[0.2, 0.27, 0.27, 0.22])
                t.auto_set_font_size(False)
                t.set_fontsize(7)
            tab.text(0, 0.02, "Make: " + ", ".join(names) + "\nLaser/waterjet from the DXF of the same name.\n"
                     "Holes for pins: ream to H8 after cutting.", fontsize=7, va="bottom")
            _title_block(fig, base, pl.material, pl.thickness, len(names), "laser cut")
            pdf.savefig(fig)
            plt.close(fig)
        fp = shell_flat_pattern()
        fig = plt.figure(figsize=(11.69, 8.27))
        ax = fig.add_axes([0.05, 0.15, 0.9, 0.75])
        L, W = fp["flat_length_mm"], fp["width_mm"]
        ax.plot([0, L, L, 0, 0], [0, 0, W, W, 0], color="#111")
        x = 0.0
        for s, a in zip(fp["segments_mm"][:-1], fp["bend_angles_deg"], strict=True):
            x += s
            ax.plot([x, x], [0, W], "--", color="#d43b2a", lw=0.8)
            ax.text(x, W + 6, f"{a:.0f}°", ha="center", fontsize=8)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(f"BK_shell flat pattern {L:.1f} x {W:.1f} mm, {len(fp['bend_angles_deg'])} bends, "
                     f"inner radius {BEND_R:g} mm, K = {SHEET_K}", fontsize=9, loc="left")
        _title_block(fig, "BK_shell", "S235", BUCKET_T, 1, "laser cut + press brake",
                     "Weld side plates and cutting edge after bending; check the profile with a printed template.")
        pdf.savefig(fig)
        plt.close(fig)


def bom_rows(parts: list[Part], masses: dict) -> list[dict]:
    rows: dict = {}
    for p in parts:
        key = p.name[:-2] if p.name[-2:] in ("_L", "_R") else p.name
        key = key.rstrip("_0123456789") if p.name.startswith("FF_cross") else key
        r = rows.setdefault(key, {"item": key, "qty": 0, "kind": p.kind, "material": p.material,
                                  "thickness_mm": p.plate.thickness if p.plate else "",
                                  "mass_each_kg": round(masses.get(p.name, 0.0), 3), "supplier_or_process": p.bom or {
                                      "plate": "laser cut from DXF", "sheet": "laser cut + bend from DXF",
                                      "turned": "machine from drawing/BOM", "printed": "3D print PETG (STL)"}.get(
                                      p.kind, ""), "note": p.note})
        r["qty"] += p.qty
    return list(rows.values())


EXTRA_BOM = [  # parts with no geometry in the model
    ("ODrive v3.6 56 V (or compatible), dual-axis motor controller", 2, "motor control, CAN to the gateway"),
    ("Teensy 4.1 + SN65HVD230 CAN transceiver x2", 1, "drive-by-wire gateway"),
    ("Teensy 4.0 + SN65HVD230 CAN transceiver", 1, "sensor node: PLC's own speed, articulation and boom sensors"),
    ("AS5600 magnetic angle sensor board + diametric magnet", 2, "articulation pin and boom pivot (sensor node)"),
    ("Raspberry Pi 4 (4 GB) + Waveshare RS485 CAN HAT (MCP2515)", 1, "safety PLC stand-in, CODESYS Control SL"),
    ("8-ch 24 V opto-isolated input board + 8-ch relay board (5 V coil)", 1, "PLC I/O"),
    ("Pilz PNOZ s4 24 V DC safety relay (or equivalent cat. 3 relay)", 1, "e-stop chain, K-DRIVE and K-IMPL"),
    ("Finder 60.13 3-pole relay 24 V DC + socket", 4, "K-DRIVE: per wheel, phase disconnect + short (dynamic brake)"),
    ("Finder 60.12 2-pole relay 24 V DC + socket", 1, "K-IMPL: actuator supply"),
    ("Cytron MDD10A dual motor driver", 3, "five PA-14P actuators (lift L/R, tilt, steer L/R)"),
    ("DC-DC 36 V -> 24 V 10 A, 36 V -> 12 V 10 A, 24 V -> 5 V 5 A", 1, "power"),
    ("Radio remote e-stop with fail-safe relay output", 1, "remote e-stop in the chain"),
    ("2D lidar (e.g. LDROBOT LD19) x2", 1, "evaluator profile stand-in for the safety scanners"),
    ("Raspberry Pi 5 8 GB or Jetson Orin Nano", 1, "autonomy computer, ROS 2 Jazzy"),
    ("u-blox ZED-F9P RTK board + antenna", 1, "localisation outdoors"),
    ("Fasteners: M5 x 12 button head (60), M4 (40), 10 h8 pins with e-clips, flanged bronze bushes 10x12", 1,
     "see the assembly STEP for positions"),
]


def renders(parts: list[Part], out_dir: Path) -> list[Path]:
    """Shaded views from tessellated geometry (no CAD license needed to look at them)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.colors import to_rgb
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    views = [("carry_iso", CARRY, (22, -55)), ("dump_side", dict(gamma=0.0, boom=0.85, bucket=-0.7, osc=0.0), (0, -90)),
             ("turn_top", dict(gamma=0.69, boom=-0.2, bucket=0.55, osc=0.0), (89, -90)),
             ("dig_iso", dict(gamma=0.0, boom=-0.62, bucket=0.0, osc=0.0), (18, -130))]
    paths = []
    light = np.array([0.4, -0.5, 0.8])
    light /= np.linalg.norm(light)
    for name, pose, (elev, azim) in views:
        fig = plt.figure(figsize=(8, 6), dpi=130)
        ax = fig.add_subplot(projection="3d")
        for p, shp in placed(build(pose), pose):
            verts, tris = shp.tessellate(1.0, 0.5)
            if not tris:
                continue
            V = np.array([[v.x, v.y, v.z] for v in verts])
            T = np.array(tris)
            polys = V[T]
            n = np.cross(polys[:, 1] - polys[:, 0], polys[:, 2] - polys[:, 0])
            n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
            shade = 0.45 + 0.55 * np.clip(np.abs(n @ light), 0, 1)
            base = np.array(to_rgb(COLORS[p.color]))
            ax.add_collection3d(Poly3DCollection(polys, facecolors=np.clip(base * shade[:, None], 0, 1),
                                                 edgecolor="none"))
        ax.set_xlim(-720, 720)
        ax.set_ylim(-400, 400)
        ax.set_zlim(0, 720)
        ax.set_box_aspect((1440, 800, 720), zoom=1.35)
        ax.view_init(elev, azim)
        ax.axis("off")
        fig.tight_layout()
        path = out_dir / f"render_{name}.png"
        fig.savefig(path)
        plt.close(fig)
        paths.append(path)
    return paths


def export(parts: list[Part], report: dict) -> None:
    import csv
    for sub in ("step", "dxf", "stl"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)
    for f in OUT.glob("*/*"):
        f.unlink()
    pl = dict((p.name, s) for p, s in placed(parts, CARRY))
    assy = cq.Assembly(name="kauha_scale8")
    for p in parts:
        shp = pl[p.name]
        if p.kind != "bought":
            cq.exporters.export(cq.Workplane("XY").add(shp), str(OUT / "step" / f"{p.name}.step"))
        if p.kind == "printed":
            cq.exporters.export(cq.Workplane("XY").add(shp), str(OUT / "stl" / f"{p.name}.stl"), tolerance=0.05)
        assy.add(cq.Workplane("XY").add(shp), name=p.name, color=cq.Color(COLORS[p.color]))
    assy.export(str(OUT / "kauha_scale8_assembly.step"))
    for base, plate, _names in plate_groups(parts):
        write_dxf(base, plate, OUT / "dxf" / f"{base}.dxf")
    report["bucket_shell_flat"] = write_shell_dxf(OUT / "dxf" / "BK_shell_flat.dxf")
    imgs = renders(parts, OUT)
    write_drawings(parts, OUT / "kauha_scale8_drawings.pdf", imgs)
    _, _, mrows = mass_properties(parts)
    masses = dict(mrows)
    rows = bom_rows(parts, masses)
    with open(OUT / "bom.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
        for desc, qty, note in EXTRA_BOM:
            w.writerow({"item": desc, "qty": qty, "kind": "bought", "material": "", "thickness_mm": "",
                        "mass_each_kg": "", "supplier_or_process": "", "note": note})
    (OUT / "report.json").write_text(json.dumps(report, indent=1, default=str))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="run the checks only")
    a = ap.parse_args()
    parts = build(CARRY)
    rep = check(parts)
    problems = [*rep["ground"], *rep["actuators"]] + [f"{k}: {v}" for k, v in rep["interference"].items()]
    st = rep["stability"]
    sand = rep["bucket_struck_volume_l"] * 1.6
    if st["rated_operating_load_kg"] < sand:
        problems.append(f"rated operating load {st['rated_operating_load_kg']} kg < bucket of sand {sand:.1f} kg")
    print(f"{len(parts)} parts, {len(POSES)} poses checked; mass {st['mass_kg']} kg, tipping load at full turn "
          f"{st['tipping_load_full_turn_kg']} kg, rated operating load {st['rated_operating_load_kg']} kg, "
          f"bucket {rep['bucket_struck_volume_l']} L")
    if problems:
        print("problems:", *problems, sep="\n  ")
        raise SystemExit(1)
    print("checks: pass")
    if not a.check:
        export(parts, rep)
        print(f"exported to {OUT}")


if __name__ == "__main__":
    main()
