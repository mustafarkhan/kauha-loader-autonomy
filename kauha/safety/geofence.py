"""Safety-side position and geofence.

The safety PLC has its own GNSS receiver (not RTK) plus its own speed and
articulation sensors. Between fixes it dead-reckons and grows a conservative
uncertainty bound. A zone check passes only if the position is inside the
zone by more than the bound (SR-07). The position is reported invalid once
the bound exceeds `max_uncertainty_m`; in AUTO the PLC then stops the
machine after a short grace time.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from kauha.geometry import dist_to_poly_edge, point_in_poly

Polygon = list[tuple[float, float]]


@dataclass
class SafetyPositioner:
    lf: float
    lr: float
    fix_sigma_m: float = 0.3
    drift_per_m: float = 0.02       # odometry + gyro dead-reckoning drift bound
    max_uncertainty_m: float = 3.0
    x: float = 0.0
    y: float = 0.0
    th: float = 0.0
    bound: float = 99.0

    def predict(self, v: float, gamma: float, gamma_rate: float, dt: float) -> None:
        th_dot = (v * math.sin(gamma) + self.lr * gamma_rate) / (self.lf * math.cos(gamma) + self.lr)
        self.th += th_dot * dt
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.bound += self.drift_per_m * abs(v) * dt

    def fix(self, x: float, y: float, th: float) -> None:
        self.x, self.y, self.th = x, y, th
        self.bound = 3 * self.fix_sigma_m

    @property
    def valid(self) -> bool:
        return self.bound <= self.max_uncertainty_m

    def inside(self, poly: Polygon) -> bool:
        return point_in_poly(self.x, self.y, poly) and dist_to_poly_edge(self.x, self.y, poly) > self.bound


@dataclass
class Geofence:
    operating_area: Polygon
    dig_zones: dict[str, Polygon]
    dock_zone: Polygon
    pos: SafetyPositioner = field(default=None)  # type: ignore[assignment]

    def evaluate(self) -> dict:
        p = self.pos
        return {
            "position_valid": p.valid,
            "in_operating_area": p.inside(self.operating_area),
            # Muting/dock zones only need the point inside the zone; the zone
            # polygons are drawn with the uncertainty margin already included.
            "in_dig_zone": p.valid and any(point_in_poly(p.x, p.y, z) for z in self.dig_zones.values()),
            "in_dock_zone": p.valid and point_in_poly(p.x, p.y, self.dock_zone),
        }
