"""Path generation and articulated pure-pursuit tracking.

Paths are cubic Bezier curves between poses, with the handle length searched
so that the curvature stays inside what the articulation can achieve.
Forward segments are planned for the front axle; reverse segments for the
rear axle, which leads when reversing (tracking the trailing axle in reverse
is poorly conditioned).

Steady-state curvature of an articulated vehicle at the leading axle:
    kappa = sin(gamma) / (lf cos(gamma) + lr)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def kappa_from_gamma(g: float, lf: float, lr: float) -> float:
    return math.sin(g) / (lf * math.cos(g) + lr)


def gamma_from_kappa(k: float, lf: float, lr: float, g_max: float) -> float:
    """Invert kappa(gamma) by Newton iteration (exact for any lf, lr)."""
    g = 2 * math.atan(k * (lf + lr) / 2)
    for _ in range(6):
        f = math.sin(g) - k * (lf * math.cos(g) + lr)
        df = math.cos(g) + k * lf * math.sin(g)
        g -= f / df
    return max(-g_max, min(g_max, g))


@dataclass
class Path:
    pts: np.ndarray        # (N,2)
    heading: np.ndarray    # (N,) direction of travel
    s: np.ndarray          # arc length
    kappa: np.ndarray
    reverse: bool
    feasible: bool

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def nearest(self, x: float, y: float, hint: int = 0, window: int = 80) -> int:
        lo, hi = max(0, hint - 5), min(len(self.pts), hint + window)
        d = np.hypot(self.pts[lo:hi, 0] - x, self.pts[lo:hi, 1] - y)
        return lo + int(np.argmin(d))

    def at_s(self, s: float) -> int:
        return int(min(len(self.s) - 1, np.searchsorted(self.s, s)))


def bezier(p0, h0, p1, h1, kappa_max: float, reverse: bool, n: int = 60) -> Path:
    """Cubic Bezier from p0 (travel direction h0) to p1 (travel direction h1)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    dist = float(np.hypot(*(p1 - p0)))
    best = None
    for frac in (0.35, 0.45, 0.55, 0.3, 0.65, 0.25, 0.75, 0.2, 0.85):
        d = max(0.5, frac * dist)
        c1 = p0 + d * np.array([math.cos(h0), math.sin(h0)])
        c2 = p1 - d * np.array([math.cos(h1), math.sin(h1)])
        t = np.linspace(0, 1, n)[:, None]
        pts = (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * c1 + 3 * (1 - t) * t ** 2 * c2 + t ** 3 * p1
        d1 = 3 * (1 - t) ** 2 * (c1 - p0) + 6 * (1 - t) * t * (c2 - c1) + 3 * t ** 2 * (p1 - c2)
        d2 = 6 * (1 - t) * (c2 - 2 * c1 + p0) + 6 * t * (p1 - 2 * c2 + c1)
        sp = np.hypot(d1[:, 0], d1[:, 1])
        k = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(sp, 1e-6) ** 3
        kmax = float(np.abs(k).max())
        if best is None or kmax < best[0]:
            best = (kmax, pts, d1, k)
        if kmax <= kappa_max:
            break
    kmax, pts, d1, k = best
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    s = np.concatenate([[0], np.cumsum(seg)])
    hd = np.arctan2(d1[:, 1], d1[:, 0])
    return Path(pts, hd, s, k, reverse, kmax <= kappa_max)


def straight(p0, heading: float, length: float, reverse: bool, n: int = 30) -> Path:
    t = np.linspace(0, length, n)
    pts = np.column_stack([p0[0] + t * math.cos(heading), p0[1] + t * math.sin(heading)])
    return Path(pts, np.full(n, heading), t, np.zeros(n), reverse, True)


@dataclass
class PurePursuit:
    lf: float
    lr: float
    g_max: float
    idx: int = 0

    def reset(self) -> None:
        self.idx = 0

    def steer(self, path: Path, ref_xy: tuple[float, float], travel_heading: float, v: float) -> tuple[float, float]:
        """Return (articulation command, remaining arc length)."""
        self.idx = path.nearest(ref_xy[0], ref_xy[1], self.idx)
        ld = min(6.0, max(2.5, 2.0 + 0.9 * abs(v)))
        j = path.at_s(path.s[self.idx] + ld)
        tx, ty = path.pts[j]
        remaining = path.length - float(path.s[self.idx])
        if j == len(path.pts) - 1:
            # extend the final heading so the lookahead never collapses
            ext = ld - (path.length - path.s[self.idx])
            tx += max(0.0, ext) * math.cos(path.heading[-1])
            ty += max(0.0, ext) * math.sin(path.heading[-1])
        alpha = math.atan2(ty - ref_xy[1], tx - ref_xy[0]) - travel_heading
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        k = 2 * math.sin(alpha) / ld
        g = gamma_from_kappa(k, self.lf, self.lr, self.g_max)
        # Reversing on the rear axle: positive articulation curves the rear to its right.
        return (-g if path.reverse else g), remaining


def from_points(pts: np.ndarray, reverse: bool) -> Path:
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    s = np.concatenate([[0], np.cumsum(seg)])
    d = np.gradient(pts, axis=0)
    hd = np.arctan2(d[:, 1], d[:, 0])
    dh = np.gradient(np.unwrap(hd))
    k = dh / np.maximum(np.gradient(s), 1e-6)
    return Path(pts, hd, s, k, reverse, True)


def reverse_arc(front_pose: tuple[float, float, float], d_heading: float, wheelbase: float,
                radius: float = 6.0, straight_m: float = 2.0, runout_m: float = 3.0,
                ds: float = 0.2) -> tuple[Path, tuple[float, float, float]]:
    """Reverse out along a straight then a constant-radius arc (rear-axle path).

    Returns the rear-axle path and the front-axle pose at its end (articulation
    straightened). This is the backing half of the loader's V-cycle.
    """
    x, y, th = front_pose
    beta = th + math.pi
    rx, ry = x - wheelbase * math.cos(th), y - wheelbase * math.sin(th)
    pts = [(rx, ry)]
    n_s = max(1, int(straight_m / ds))
    for _ in range(n_s):
        rx += ds * math.cos(beta)
        ry += ds * math.sin(beta)
        pts.append((rx, ry))
    n_a = max(1, int(abs(d_heading) * radius / ds))
    step = d_heading / n_a
    for _ in range(n_a):
        beta += step
        rx += (abs(step) * radius) * math.cos(beta)
        ry += (abs(step) * radius) * math.sin(beta)
        pts.append((rx, ry))
    # straight run-out lets the articulation return to centre before the stop
    for _ in range(max(1, int(runout_m / ds))):
        rx += ds * math.cos(beta)
        ry += ds * math.sin(beta)
        pts.append((rx, ry))
    th_end = beta - math.pi
    end = (rx + wheelbase * math.cos(th_end), ry + wheelbase * math.sin(th_end),
           math.atan2(math.sin(th_end), math.cos(th_end)))
    return from_points(np.array(pts), reverse=True), end
