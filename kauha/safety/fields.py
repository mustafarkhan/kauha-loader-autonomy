"""Safety-scanner field sets.

Field sets are computed offline and loaded into the scanners, the way
commercial safety laser scanners work: the safety PLC only *selects* a set
through its inputs; the scanner evaluates the selected protective and warning
fields and switches its OSSD outputs.

A set is chosen by (direction, speed band, steering sector), plus two special
sets that are only allowed inside configured zones:

  DIG   front protective field muted to the bucket contour, side fields kept,
        speed capped; only inside a dig zone and for a limited time.
  DOCK  short front protective field for the final approach to a truck or
        hopper, speed capped; only inside a dock zone.

Fields are polygons in the frame of the scanner-carrying body: the front
frame for the front scanners (origin at the front axle centre, x forward),
the rear frame for the rear scanners (origin at the rear axle centre, x
pointing *backwards*).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from kauha.safety.stopping import StopBudget, max_speed_for_field, stopping_distance

Polygon = list[tuple[float, float]]

SCANNER_PROTECTIVE_RANGE_M = 4.0   # to confirm against the selected scanner
SCANNER_WARNING_RANGE_M = 10.0
SIDE_MARGIN_M = 0.5


@dataclass(frozen=True)
class FieldSet:
    id: int
    name: str
    direction: int            # +1 forward, -1 reverse, 0 standstill
    v_max: float              # top of the speed band this set protects
    protective: Polygon       # in the leading body's frame
    warning: Polygon
    side: Polygon = field(default_factory=list)   # mirrored on both sides of the front frame
    side_rear: Polygon = field(default_factory=list)   # mirrored on both sides of the rear frame (moving sets)
    # SR-12 contact bands: a narrow strip beside each frame, evaluated at the same
    # time as the active set (a second simultaneous field on the same scanners).
    # Teleop recovery from a side stop is refused while a band is occupied.
    side_contact: Polygon = field(default_factory=lambda: contact_band("front"))
    side_contact_rear: Polygon = field(default_factory=lambda: contact_band("rear"))
    # Standstill only: the rear frame has its own contour, so it gets its own fields.
    rear_protective: Polygon | None = None
    rear_warning: Polygon | None = None

    def protective_for(self, body: str) -> Polygon:
        return self.rear_protective if body == "rear" and self.rear_protective is not None else self.protective

    def warning_for(self, body: str) -> Polygon:
        return self.rear_warning if body == "rear" and self.rear_warning is not None else self.warning


@dataclass(frozen=True)
class Geometry:
    front_contour_x: float = 5.1     # furthest bucket reach (carry posture) ahead of front axle
    rear_contour_x: float = 1.7      # counterweight, behind rear axle (rear-frame x)
    half_width: float = 1.4          # bucket is the widest part (2.7 m)
    wheelbase_half: float = 1.6      # lf = lr
    rear_half_width: float = 1.3     # rear frame (2.55 m wide) plus a little
    scanner_x: float = 2.0           # front scanners on the front-frame fenders
    dock_field_end_x: float = 2.8    # truck side must stay beyond this when docked
    dig_side_end_x: float = 1.0      # DIG side fields cover the cab/articulation area only;
                                     # further forward the pile flanks beside the bucket
                                     # would trip them on every pass


CONTACT_BAND_M = 0.35


def contact_band(body: str, g: "Geometry | None" = None) -> Polygon:
    """Strip beside one frame (positive-y side; mirrored by the evaluator)."""
    g = g or Geometry()
    if body == "front":
        return [(0.0, g.half_width), (g.front_contour_x, g.half_width),
                (g.front_contour_x, g.half_width + CONTACT_BAND_M), (0.0, g.half_width + CONTACT_BAND_M)]
    y0, y1 = g.rear_half_width, g.rear_half_width + CONTACT_BAND_M
    return [(-g.wheelbase_half, y0), (g.rear_contour_x, y0), (g.rear_contour_x, y1), (-g.wheelbase_half, y1)]


def _arc_edges(x0: float, length: float, half_w: float, kappa: float, n: int = 8):
    left, right = [], []
    for i in range(n + 1):
        s = length * i / n
        if abs(kappa) < 1e-6:
            cx, cy, th = x0 + s, 0.0, 0.0
        else:
            th = kappa * s
            cx = x0 + math.sin(th) / kappa
            cy = (1 - math.cos(th)) / kappa
        nx, ny = -math.sin(th), math.cos(th)
        left.append((cx + nx * half_w, cy + ny * half_w))
        right.append((cx - nx * half_w, cy - ny * half_w))
    return left, right


def _swept(x0: float, length: float, half_w: float, kappa: float, n: int = 8) -> Polygon:
    """Area swept ahead of the contour along an arc of curvature kappa."""
    left, right = _arc_edges(x0, length, half_w, kappa, n)
    return left + right[::-1]


def _convex_hull(pts: list[tuple[float, float]]) -> Polygon:
    pts = sorted(set(pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _swept_envelope(x0: float, length: float, half_w: float, k_lo: float, k_hi: float, n: int = 8) -> Polygon:
    """Convex envelope of the areas swept for every curvature in [k_lo, k_hi].

    Covers the machine's path for any articulation inside the sector, including
    steering during the stop. Convexity over-covers the inside of the turn a
    little (more nuisance stops), never under-covers.
    """
    pts: list[tuple[float, float]] = []
    for i in range(7):
        k = k_lo + (k_hi - k_lo) * i / 6
        left, right = _arc_edges(x0, length, half_w, k, n)
        pts += left + right
    return _convex_hull(pts)


def kappa_of(gamma: float, lf: float = 1.6, lr: float = 1.6) -> float:
    return math.sin(gamma) / (lf * math.cos(gamma) + lr)


# Speed bands: (upper speed m/s). Field length follows from the stop budget.
BANDS = (0.5, 1.5, 2.5, 3.0)
# Articulation sectors (rad). Each field covers the whole sector (envelope), up
# to the machine's full articulation in the outer sectors.
SECTORS = (("R2", -0.70, -0.30), ("R1", -0.30, -0.10), ("S", -0.10, 0.10), ("L1", 0.10, 0.30), ("L2", 0.30, 0.70))
DIG_SPEED = 1.0
DOCK_SPEED = 0.6
STANDSTILL = 0.1


def build_field_sets(b: StopBudget, g: Geometry | None = None) -> dict[int, FieldSet]:
    g = g or Geometry()
    sets: dict[int, FieldSet] = {}
    hw = g.half_width + SIDE_MARGIN_M
    side = [(0.0, g.half_width), (g.front_contour_x, g.half_width),
            (g.front_contour_x, g.half_width + 0.6), (0.0, g.half_width + 0.6)]

    def lengths(v: float) -> tuple[float, float]:
        p = min(SCANNER_PROTECTIVE_RANGE_M, max(1.0, stopping_distance(v, b)))
        w = min(SCANNER_WARNING_RANGE_M, p + 2.0 + 1.5 * v)
        return p, w

    # Rear-frame sides, in the rear frame (x backwards from the rear axle): from the
    # articulation joint to the counterweight. A person beside the rear frame is
    # where an articulating loader pinches and where reversing passes close.
    rear_side = [(-g.wheelbase_half, g.rear_half_width), (g.rear_contour_x, g.rear_half_width),
                 (g.rear_contour_x, g.rear_half_width + 0.6), (-g.wheelbase_half, g.rear_half_width + 0.6)]
    p, w = lengths(STANDSTILL)
    sets[0] = FieldSet(0, "standstill", 0, STANDSTILL,
                       _swept(g.front_contour_x, p, hw, 0.0), _swept(g.front_contour_x, w, hw, 0.0), side,
                       side_rear=rear_side, rear_protective=_swept(g.rear_contour_x, p, hw, 0.0),
                       rear_warning=_swept(g.rear_contour_x, w, hw, 0.0))
    fid = 1
    for direction in (1, -1):
        x0 = g.front_contour_x if direction > 0 else g.rear_contour_x
        for v in BANDS:
            p, w = lengths(v)
            for label, g_lo, g_hi in SECTORS:
                sign = 1 if direction > 0 else -1
                ks = sorted((kappa_of(g_lo) * sign, kappa_of(g_hi) * sign))
                # suffix = articulation sector, not the turn direction of the body
                name = f"{'fwd' if direction > 0 else 'rev'}_{v:.1f}_{label}"
                sets[fid] = FieldSet(fid, name, direction, v, _swept_envelope(x0, p, hw, *ks),
                                     _swept_envelope(x0, w, hw, *ks), side if direction > 0 else [],
                                     side_rear=rear_side)
                fid += 1
    # DIG: the front protective field is muted (the bucket is meant to enter the
    # pile); side fields stop short of the bucket so the pile flanks do not trip
    # them. People protection in the pile face relies on the dig zone being an
    # access-controlled exclusion zone, the speed cap and the time limit.
    dig_side = [(0.0, g.half_width), (g.dig_side_end_x, g.half_width),
                (g.dig_side_end_x, g.half_width + 0.6), (0.0, g.half_width + 0.6)]
    sets[100] = FieldSet(100, "dig", 1, DIG_SPEED, [], [], dig_side)
    # DOCK: bucket raised over the truck body; the protective field covers the
    # ground between the front tyres and the truck side.
    dock = [(g.scanner_x, -hw), (g.dock_field_end_x, -hw), (g.dock_field_end_x, hw), (g.scanner_x, hw)]
    dock_side = [(0.0, g.half_width), (g.dock_field_end_x, g.half_width),
                 (g.dock_field_end_x, g.half_width + 0.6), (0.0, g.half_width + 0.6)]
    sets[101] = FieldSet(101, "dock", 1, DOCK_SPEED, dock, dock, dock_side)
    return sets


def select_field_set(sets: dict[int, FieldSet], v: float, gamma: float, dig: bool, dock: bool) -> int:
    """Pure selection rule implemented by the safety PLC each cycle."""
    if dig and v > -STANDSTILL:      # reversing out of the pile uses the rear fields
        return 100
    if dock and v > -STANDSTILL:     # reversing away from the truck uses the rear fields
        return 101
    if abs(v) < STANDSTILL:
        return 0
    direction = 1 if v > 0 else -1
    # Sector follows the articulation sign; the reverse sets are built with the
    # mirrored curvature, so no flip is needed here.
    sector = next((i for i, (_, lo, hi) in enumerate(SECTORS) if lo <= gamma < hi),
                  0 if gamma < 0 else len(SECTORS) - 1)
    band = next((i for i, top in enumerate(BANDS) if abs(v) <= top), len(BANDS) - 1)
    n = len(SECTORS)
    base = 1 if direction > 0 else 1 + len(BANDS) * n
    return base + band * n + sector


def point_in_polygon(x: float, y: float, poly: Polygon) -> bool:
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside


def field_speed_cap(b: StopBudget) -> float:
    """Highest speed the scanners can protect with the available field range."""
    return max_speed_for_field(SCANNER_PROTECTIVE_RANGE_M, b)
