"""EKF localisation of the front-axle pose (x, y, heading).

Prediction: wheel odometry speed and the front-frame gyro.
Updates: dual-antenna RTK GNSS (position + heading) when available, and
range/bearing to surveyed retroreflective posts from the 3D lidar. The posts
keep the estimate bounded through GNSS outages (pile shadows, buildings,
jamming); the supervisor slows or stops the machine as the covariance grows.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class EKF:
    x: np.ndarray = field(default_factory=lambda: np.zeros(3))
    P: np.ndarray = field(default_factory=lambda: np.diag([1.0, 1.0, 0.1]))
    q_v: float = 0.05          # speed noise std (m/s), inflated for slip
    q_w: float = 0.01          # gyro noise std (rad/s)
    landmarks: list[tuple[float, float]] = field(default_factory=list)
    gate: float = 9.21         # chi-square 99 % for 2 dof
    rejected: int = 0
    gnss_rejects: int = 0      # consecutive
    gnss_resets: int = 0
    reset_after: int = 10      # consecutive RTK rejections (1 s at 10 Hz)

    def init(self, x: float, y: float, th: float, sigma: float = 0.1) -> None:
        self.x = np.array([x, y, th])
        self.P = np.diag([sigma ** 2, sigma ** 2, 0.01])

    def predict(self, v: float, w: float, dt: float, slipping: bool = False) -> None:
        x, y, th = self.x
        self.x = np.array([x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, wrap(th + w * dt)])
        F = np.array([[1, 0, -v * math.sin(th) * dt], [0, 1, v * math.cos(th) * dt], [0, 0, 1]])
        G = np.array([[math.cos(th) * dt, 0], [math.sin(th) * dt, 0], [0, dt]])
        # Wheel odometry is unreliable while the tyres spin in the pile: trust it less.
        q_v = self.q_v + (0.3 if slipping else 0.03) * abs(v)
        Q = np.diag([q_v ** 2, self.q_w ** 2])
        self.P = F @ self.P @ F.T + G @ Q @ G.T

    def _update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, R: np.ndarray, angle_idx=()) -> bool:
        y = z - h
        for i in angle_idx:
            y[i] = wrap(y[i])
        S = H @ self.P @ H.T + R
        d2 = float(y @ np.linalg.solve(S, y))
        if d2 > self.gate * (len(z) / 2):
            self.rejected += 1
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T     # Joseph form
        return True

    def update_gnss(self, gx: float, gy: float, gth: float, s_xy: float, s_th: float, rtk: bool = True) -> bool:
        H = np.eye(3)
        R = np.diag([s_xy ** 2, s_xy ** 2, s_th ** 2])
        ok = self._update(np.array([gx, gy, gth]), self.x.copy(), H, R, angle_idx=(2,))
        self.gnss_rejects = 0 if ok else self.gnss_rejects + 1
        if not ok and rtk and self.gnss_rejects >= self.reset_after:
            # Persistent disagreement with a fixed RTK solution means the filter,
            # not the receiver, is wrong: re-initialise from GNSS.
            self.x = np.array([gx, gy, gth])
            self.P = np.diag([s_xy ** 2 * 4, s_xy ** 2 * 4, s_th ** 2 * 4])
            self.gnss_resets += 1
            self.gnss_rejects = 0
        return ok

    def update_landmark(self, lid: int, r: float, b: float, s_r: float = 0.05, s_b: float = 0.006) -> bool:
        lx, ly = self.landmarks[lid]
        x, y, th = self.x
        dx, dy = lx - x, ly - y
        q = dx * dx + dy * dy
        rr = math.sqrt(q)
        h = np.array([rr, wrap(math.atan2(dy, dx) - th)])
        H = np.array([[-dx / rr, -dy / rr, 0], [dy / q, -dx / q, -1]])
        return self._update(np.array([r, b]), h, H, np.diag([s_r ** 2, s_b ** 2]), angle_idx=(1,))

    @property
    def sigma_xy(self) -> float:
        return math.sqrt(max(self.P[0, 0], self.P[1, 1]))
