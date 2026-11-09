"""Software field evaluator for the testbed's evaluator profile (NOT a safety function).

Stands in for the safety scanners' own field evaluation until rated scanners are
fitted: takes 2D lidar points, the field set the PLC selected (from SafetyStatus),
and the testbed's field sets (hardware/scale/generated/fields_scale8_evaluator.json),
and returns the ScannerStatus bits the PLC reads. The geometry is the same code
path the twin's ray-cast scanners use (kauha.safety.fields.point_in_polygon).

Limits, stated so they are not forgotten: a hobby lidar has no certified response
time, detection capability or self-test, and this code runs on a non-safety
computer. The PLC treats a stale frame or a wrong set echo as a fault, but people
must stay outside the test area while the evaluator profile is in use
(docs/scale/test_plan.md, rule 1).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from kauha.config import REPO_ROOT
from kauha.safety.fields import point_in_polygon

FIELDS = REPO_ROOT / "hardware" / "scale" / "generated" / "fields_scale8_evaluator.json"
MIN_POINTS = 2          # a field counts as occupied when at least this many points fall inside


@dataclass(frozen=True)
class Mount:
    """Lidar pose on its body: position (m) in the body frame and yaw (rad)."""
    x: float
    y: float
    yaw: float
    body: str             # "front" or "rear" (rear frame: x points backwards from the rear axle)


def load_sets(path: Path = FIELDS) -> dict[int, dict]:
    return {int(k): v for k, v in json.loads(path.read_text()).items()}


def to_body(m: Mount, ranges: list[float], angle_min: float, angle_inc: float, max_range: float = 3.0):
    c, s = math.cos(m.yaw), math.sin(m.yaw)
    for i, r in enumerate(ranges):
        if not (0.02 < r < max_range):
            continue
        a = angle_min + i * angle_inc
        px, py = r * math.cos(a), r * math.sin(a)
        yield m.x + c * px - s * py, m.y + s * px + c * py


def _count(points, poly, mirror=False) -> int:
    if not poly:
        return 0
    n = 0
    for x, y in points:
        if point_in_polygon(x, y, poly) or (mirror and point_in_polygon(x, -y, poly)):
            n += 1
    return n


def evaluate(sets: dict[int, dict], set_id: int, front_pts: list, rear_pts: list, boom_raised: bool) -> dict:
    """ScannerStatus signals for one scan pair. Side and contact fields are mirrored (both sides)."""
    fs = sets.get(set_id)
    if fs is None:
        return {"ScannersOk": 0, "EvaluatedSet": set_id & 0xFF}
    prot_f = fs["protective"] if fs["direction"] >= 0 else []
    prot_r = fs["rear_protective"] or (fs["protective"] if fs["direction"] <= 0 else [])
    warn_f = fs["warning"] if fs["direction"] >= 0 else []
    warn_r = fs["rear_warning"] or (fs["warning"] if fs["direction"] <= 0 else [])
    hit = lambda pts, poly, mirror=False: _count(pts, poly, mirror) >= MIN_POINTS  # noqa: E731
    return {
        "FrontProt": int(hit(front_pts, prot_f)), "FrontWarn": int(hit(front_pts, warn_f)),
        "RearProt": int(hit(rear_pts, prot_r)), "RearWarn": int(hit(rear_pts, warn_r)),
        "SideProt": int(hit(front_pts, fs["side"], True) or hit(rear_pts, fs["side_rear"], True)),
        "SideContact": int(hit(front_pts, fs["side_contact"], True) or hit(rear_pts, fs["side_contact_rear"], True)),
        "FrontSide": int(hit(front_pts, fs["front_side"], True)),
        "UnderNear": int(hit(front_pts, fs["under_near"])), "UnderMid": int(hit(front_pts, fs["under_mid"])),
        "UnderFar": int(hit(front_pts, fs["under_far"]) and boom_raised),
        "ScannersOk": 1, "EvaluatedSet": set_id & 0xFF,
    }
