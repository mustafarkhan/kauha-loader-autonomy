"""Rated coverage of the protective sensors, and the check that every field lies inside it.

A protective field only protects where a safety-rated sensor is rated to detect.
The fields in ``kauha.safety.fields`` start at the machine contour (the bucket's
front edge is 5.1 m ahead of the front axle), but the sensors sit on the frames,
about 3 m behind it. An outdoor safety laser scanner is rated to 4 m, so it cannot
watch a field that ends 8 m away. This module states what each sensor is rated for
and checks the field sets against it.

Sensor classes (catalogue data, to confirm for the parts finally bought):

  scanner  outdoor safety laser scanner, PL d / category 3, protective field
           range 4 m, 275 degrees (SICK outdoorScan3 data sheet: 4 m, 0.39 degree
           resolution, 115 ms response, PFHd 8.0e-8 1/h, rain to 10 mm/h, snow to
           3 mm/h, fog down to 50 m visibility). Watches the side, contact and
           under-bucket fields and the near part of the front and rear fields.
  radar    safety radar, PL d / category 3, 100 degrees to 5 m and 40 degrees
           from 5 m to 9 m, 20 degrees vertical, up to six sensors on one control
           unit (Pilz PSEN rd1.2 F-FOV LR / Inxpect S201A-MLR product data).
           Watches the far part of the front and rear fields. Rated for detecting
           people; its use on a moving outdoor machine, its response time and its
           PFHd have to be confirmed with the manufacturer before purchase.

Three radars on each end (one straight, two turned 25 degrees outward) cover every
front and rear protective field of every field set; ``uncovered`` proves it by
sampling. Without the radars the 4 m scanners alone allow about 0.3 m/s forward
(``scanner_only_speed_cap``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from kauha.safety.fields import FieldSet, point_in_polygon
from kauha.safety.stopping import StopBudget, max_speed_for_field

SCANNER_ZONES = ((4.0, math.radians(137.5)),)
RADAR_ZONES = ((5.0, math.radians(50.0)), (9.0, math.radians(20.0)))
RADAR_SIDE_YAW = math.radians(25.0)


@dataclass(frozen=True)
class RatedSensor:
    name: str
    kind: str             # "scanner" or "radar"
    body: str             # "front" or "rear" (rear frame: x points backwards)
    x: float
    y: float
    heading: float
    zones: tuple          # ((range m, half field of view rad), ...): rated where any zone holds

    def rated(self, rng, bearing):
        """True where a return at range ``rng`` and bearing (relative to the sensor axis) is inside
        the rated coverage. Works on floats and numpy arrays."""
        ok = False
        for r, half in self.zones:
            ok = ok | ((rng <= r) & (abs(bearing) <= half))
        return ok

    def covers(self, px: float, py: float) -> bool:
        d = math.hypot(px - self.x, py - self.y)
        b = (math.atan2(py - self.y, px - self.x) - self.heading + math.pi) % (2 * math.pi) - math.pi
        return bool(self.rated(d, b))


def _end(body: str, x: float, y_scan: float, y_radar: float, flip: int) -> tuple:
    a = math.radians(45) * flip
    return (RatedSensor(f"{body}_left", "scanner", body, x, y_scan * flip, a, SCANNER_ZONES),
            RatedSensor(f"{body}_right", "scanner", body, x, -y_scan * flip, -a, SCANNER_ZONES),
            RatedSensor(f"{body}_radar_centre", "radar", body, x, 0.0, 0.0, RADAR_ZONES),
            RatedSensor(f"{body}_radar_left", "radar", body, x, y_radar, RADAR_SIDE_YAW, RADAR_ZONES),
            RatedSensor(f"{body}_radar_right", "radar", body, x, -y_radar, -RADAR_SIDE_YAW, RADAR_ZONES))


# Full-size layout. Scanner positions equal the twin's mounts (kauha.sim.sensors).
LAYOUT = _end("front", 2.0, 1.35, 1.0, 1) + _end("rear", 1.7, 1.2, 0.9, -1)


def sensors(body: str, layout: tuple = LAYOUT, kinds: tuple = ("scanner", "radar")) -> tuple:
    return tuple(s for s in layout if s.body == body and s.kind in kinds)


def _samples(poly, step: float):
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    pts = list(poly)
    nx, ny = int((max(xs) - min(xs)) / step) + 1, int((max(ys) - min(ys)) / step) + 1
    for i in range(nx + 1):
        for j in range(ny + 1):
            x, y = min(xs) + i * step, min(ys) + j * step
            if point_in_polygon(x, y, poly):
                pts.append((x, y))
    return pts


def protective_polygons(fs: FieldSet):
    """(body, field name, polygon) for every field whose violation stops the machine."""
    def both(p):
        return [p, [(x, -y) for x, y in p]] if p else []
    out = []
    if fs.direction >= 0 and fs.protective:
        out.append(("front", "protective", fs.protective))
    if fs.direction <= 0 and fs.protective_for("rear"):
        out.append(("rear", "protective", fs.protective_for("rear")))
    for name, body, poly in (("side", "front", fs.side), ("side_rear", "rear", fs.side_rear),
                             ("side_contact", "front", fs.side_contact),
                             ("side_contact_rear", "rear", fs.side_contact_rear),
                             ("front_side", "front", fs.front_side)):
        out += [(body, name, p) for p in both(poly)]
    out += [("front", n, p) for n, p in (("under_near", fs.under_near), ("under_mid", fs.under_mid),
                                         ("under_far", fs.under_far)) if p]
    return out


def uncovered(sets: dict[int, FieldSet], layout: tuple = LAYOUT, step: float = 0.15,
              kinds: tuple = ("scanner", "radar")) -> list[tuple[str, str, float, float]]:
    """Sample points of stopping fields that no rated sensor covers: (set, field, x, y)."""
    miss = []
    for fs in sets.values():
        for body, name, poly in protective_polygons(fs):
            ss = sensors(body, layout, kinds)
            miss += [(fs.name, name, x, y) for x, y in _samples(poly, step) if not any(s.covers(x, y) for s in ss)]
    return miss


def reach(sets: dict[int, FieldSet], layout: tuple = LAYOUT) -> dict[str, float]:
    """Largest distance from a front/rear protective field vertex to the nearest sensor of each kind."""
    out = {}
    for kind in ("scanner", "radar"):
        d = 0.0
        for fs in sets.values():
            for body, name, poly in protective_polygons(fs):
                if name != "protective":
                    continue
                ss = sensors(body, layout, (kind,))
                d = max(d, max(min(math.hypot(x - s.x, y - s.y) for s in ss) for x, y in poly))
        out[kind] = d
    return out


def scanner_only_speed_cap(b: StopBudget, front_contour_x: float = 5.1, half_width: float = 1.9,
                           layout: tuple = LAYOUT) -> float:
    """Forward speed the 4 m scanners alone could protect: the field may only reach as far ahead of
    the bucket as its far corners stay inside a scanner's rated range."""
    ss = sensors("front", layout, ("scanner",))
    lo, hi = 0.0, 10.0
    for _ in range(50):
        mid = (lo + hi) / 2
        pts = [(front_contour_x + mid, y) for y in (-half_width, 0.0, half_width)]
        if all(any(s.covers(x, y) for s in ss) for x, y in pts):
            lo = mid
        else:
            hi = mid
    return max_speed_for_field(lo, b)
