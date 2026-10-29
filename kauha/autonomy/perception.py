"""World model maintained by the autonomy computer.

  * PileMap: per-slot toe position of each stockpile face, initialised from
    the site survey and refined from lidar; picks the next dig slot.
  * TruckTracker: truck pose from lidar, presence from the dispatch system.
  * PeopleTracker: person detections from camera and lidar, used for the
    autonomy-level slowdown/hold *before* anyone reaches a safety field.
    This layer is a productivity and comfort function; protection is the
    safety channel's job.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class SlotEstimate:
    y_c: float
    width: float
    toe_x: float
    seen_t: float = -1.0


@dataclass
class PileMap:
    slots: dict[str, list[SlotEstimate]]
    repose_deg: dict[str, float]
    alpha: float = 0.3
    _rr: dict[str, int] = field(default_factory=dict)

    @classmethod
    def from_site(cls, site) -> "PileMap":
        return cls({n: [SlotEstimate(s.y_c, s.width, s.toe_x) for s in p.slots] for n, p in site.piles.items()},
                   {n: p.material.friction_deg for n, p in site.piles.items()})

    def observe(self, t: float, face_obs) -> None:
        for pile, j, toe, _ in face_obs:
            s = self.slots[pile][j]
            s.toe_x += self.alpha * (toe - s.toe_x)
            s.seen_t = t

    def choose(self, pile: str) -> int:
        """Most protruding slot (most material within reach); ties rotate."""
        sl = self.slots[pile]
        start = self._rr.get(pile, 0)
        order = [(start + k) % len(sl) for k in range(len(sl))]
        best = min(order, key=lambda j: (round(sl[j].toe_x, 1), order.index(j)))
        self._rr[pile] = (best + 1) % len(sl)
        return best

    def surface(self, pile: str, penetration: float) -> float:
        return max(0.0, penetration) * math.tan(math.radians(self.repose_deg[pile]))


@dataclass
class TruckTracker:
    bay: tuple[float, float, float]
    pose: tuple[float, float, float] | None = None
    seen_t: float = -1.0

    def observe(self, t: float, pose) -> None:
        if pose is not None:
            if self.pose is None:
                self.pose = pose
            else:
                self.pose = tuple(0.7 * a + 0.3 * b for a, b in zip(self.pose, pose))
            self.seen_t = t

    def best(self) -> tuple[float, float, float]:
        return self.pose if self.pose is not None else self.bay


@dataclass
class PeopleTracker:
    """Person detections confirmed over time.

    Lidar detections are taken as confirmed. A camera detection counts only
    once the detector has fired at least `confirm_hits` times within
    `confirm_radius` over the last `memory_s` seconds. The detector's measured
    false-positive rate (~0.4 per image at conf 0.25) would otherwise stop the
    machine for phantoms.
    """
    memory_s: float = 0.6
    confirm_hits: int = 3
    confirm_radius: float = 1.5
    dets: list[tuple[float, float, float]] = field(default_factory=list)
    cam: list[tuple[float, float, float]] = field(default_factory=list)

    def observe(self, t: float, points, source: str = "lidar") -> None:
        self.dets = [d for d in self.dets if t - d[0] <= self.memory_s]
        if source == "lidar":
            self.dets += [(t, x, y) for x, y in points]
            return
        self.cam = [d for d in self.cam if t - d[0] <= self.memory_s] + [(t, x, y) for x, y in points]
        for _, x, y in [d for d in self.cam if d[0] == t]:
            hits = sum(1 for _, cx, cy in self.cam if math.hypot(cx - x, cy - y) < self.confirm_radius)
            if hits >= self.confirm_hits:
                self.dets.append((t, x, y))

    def nearest_in_corridor(self, path_pts: np.ndarray, from_idx: int, half_width: float = 3.0,
                            horizon_m: float = 20.0) -> float:
        """Distance along the path to the nearest detection inside the corridor."""
        if not self.dets or path_pts is None or len(path_pts) < 2:
            return math.inf
        P = path_pts[from_idx:]
        if len(P) < 2:
            return math.inf
        seg = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
        s = np.concatenate([[0], np.cumsum(seg)])
        best = math.inf
        for _, x, y in self.dets:
            d = np.hypot(P[:, 0] - x, P[:, 1] - y)
            i = int(np.argmin(d))
            if d[i] < half_width and s[i] < horizon_m:
                best = min(best, float(s[i]))
        return best
