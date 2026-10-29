"""Field-aware speed limiting: the autonomy predicts the safety channel.

The safety PLC chooses a protective field from the measured speed and
articulation. If the planner drives at a speed whose field reaches a mapped,
static object (a stockpile flank, the truck, a wall, a survey post), the PLC
trips: a correct stop, but a nuisance one that costs a remote intervention.

The guard uses the same field-set definitions as the PLC (they are one
configuration) and the autonomy's map. For each speed band it checks whether
the band's protective field, placed at poses along the path ahead (as far as
the machine needs to slow to the next band), stays clear of mapped objects.
The planner then drives no faster than the fastest clear band.

This is a productivity function only. It never replaces the scanners: people
and unmapped objects are not in the map, and the PLC still protects against
them.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from kauha.geometry import points_in_polygon
from kauha.safety.fields import BANDS, FieldSet, build_field_sets, select_field_set
from kauha.safety.stopping import StopBudget

DECEL = 1.0
REACTION = 0.3        # command latency before the machine slows, s
RADIUS = 14.0         # only map points this close can matter


@dataclass
class FieldGuard:
    budget: StopBudget = field(default_factory=StopBudget)
    margin_scale: float = 1.0

    def __post_init__(self) -> None:
        self.sets: dict[int, FieldSet] = build_field_sets(self.budget)
        self.points = np.zeros((0, 2))

    # ------------------------------------------------------------------ map
    def set_map(self, segments: list[tuple[float, float, float, float]], posts: list[tuple[float, float]],
                spacing: float = 0.25) -> None:
        pts = [p for p in posts]
        for x0, y0, x1, y1 in segments:
            n = max(1, int(math.hypot(x1 - x0, y1 - y0) / spacing))
            for i in range(n + 1):
                u = i / n
                pts.append((x0 + u * (x1 - x0), y0 + u * (y1 - y0)))
        self.points = np.array(pts, dtype=float) if pts else np.zeros((0, 2))

    # -------------------------------------------------------------- queries
    def _clear(self, fs: FieldSet, pose: tuple[float, float, float], pts: np.ndarray) -> bool:
        if not fs.protective or len(pts) == 0:
            return True
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        lx, ly = dx * c + dy * s, -dx * s + dy * c
        if points_in_polygon(lx, ly, fs.protective).any():
            return False
        if fs.side:
            for poly in (fs.side, [(a, -b) for a, b in fs.side]):
                if points_in_polygon(lx, ly, poly).any():
                    return False
        return True

    def speed_cap(self, path_pts: np.ndarray, path_heading: np.ndarray, path_s: np.ndarray, idx: int,
                  reverse: bool, gammas: tuple[float, ...]) -> float:
        """Highest speed whose protective fields stay clear of mapped objects along the path.

        For reverse paths the points are rear-axle positions and `path_heading` the
        direction of travel, which is the rear body's backward frame the rear fields use.
        """
        if len(self.points) == 0 or path_pts is None or len(path_pts) == 0:
            return BANDS[-1]
        here = path_pts[min(idx, len(path_pts) - 1)]
        near = self.points[np.hypot(self.points[:, 0] - here[0], self.points[:, 1] - here[1]) < RADIUS]
        if len(near) == 0:
            return BANDS[-1]
        sign = -1.0 if reverse else 1.0
        lower = (0.0,) + BANDS[:-1]
        best = 0.0
        for b, top in enumerate(BANDS):
            # distance travelled while slowing from this band to the one below
            ahead = (top ** 2 - lower[b] ** 2) / (2 * DECEL) + top * REACTION
            s0 = path_s[min(idx, len(path_s) - 1)]
            ok = True
            for frac in (0.0, 0.5, 1.0):
                j = int(min(len(path_s) - 1, np.searchsorted(path_s, s0 + frac * ahead)))
                pose = (float(path_pts[j, 0]), float(path_pts[j, 1]), float(path_heading[j]))
                for g in gammas:
                    fid = select_field_set(self.sets, sign * min(top, BANDS[-1]) * 0.99, g, False, False)
                    if not self._clear(self.sets[fid], pose, near):
                        ok = False
                        break
                if not ok:
                    break
            if not ok:
                break
            best = top
        return best * 0.95
