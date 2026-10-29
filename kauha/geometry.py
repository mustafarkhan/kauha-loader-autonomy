"""Small planar geometry helpers shared by the safety channel and the twin."""
from __future__ import annotations

import math

import numpy as np

Polygon = list[tuple[float, float]]


def point_in_poly(x: float, y: float, poly: Polygon) -> bool:
    inside, j = False, len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside


def dist_to_poly_edge(x: float, y: float, poly: Polygon) -> float:
    best = math.inf
    for i in range(len(poly)):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % len(poly)]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - ax - u * dx, y - ay - u * dy))
    return best


def points_in_polygon(px: np.ndarray, py: np.ndarray, poly) -> np.ndarray:
    """Vectorised even-odd point-in-polygon test."""
    P = np.asarray(poly, dtype=float)
    xi, yi = P[:, 0], P[:, 1]
    xj, yj = np.roll(xi, 1), np.roll(yi, 1)
    Y = py[:, None]
    cond = (yi[None, :] > Y) != (yj[None, :] > Y)
    with np.errstate(divide="ignore", invalid="ignore"):
        xc = (xj - xi)[None, :] * (Y - yi[None, :]) / (yj - yi + 1e-12)[None, :] + xi[None, :]
    crossings = cond & (px[:, None] < xc)
    return (crossings.sum(axis=1) % 2) == 1
