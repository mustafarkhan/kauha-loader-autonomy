from __future__ import annotations

import math

import numpy as np
import pytest

from kauha.autonomy.localization import EKF
from kauha.autonomy.paths import PurePursuit, bezier, gamma_from_kappa, kappa_from_gamma, reverse_arc
from kauha.config import load_machine
from kauha.sim.loader import Loader, LoaderCommand, LoaderState


def test_kappa_gamma_inverse():
    for g in np.linspace(-0.6, 0.6, 13):
        assert gamma_from_kappa(kappa_from_gamma(g, 1.6, 1.4), 1.6, 1.4, 0.7) == pytest.approx(g, abs=1e-9)


def track(path, start, reverse, cfg, steps=2000):
    L = Loader(cfg, LoaderState(x=start[0], y=start[1], theta=start[2]), dt=0.025)
    pp = PurePursuit(cfg.lf, cfg.lr, cfg.max_articulation)
    worst = 0.0
    for _ in range(steps):
        if reverse:
            rx, ry, rth = L.rear_axle()
            g, rem = pp.steer(path, (rx, ry), rth + math.pi, L.state.v)
            ref = (rx, ry)
        else:
            g, rem = pp.steer(path, (L.state.x, L.state.y), L.state.theta, L.state.v)
            ref = (L.state.x, L.state.y)
        j = path.nearest(*ref, pp.idx)
        worst = max(worst, math.hypot(path.pts[j, 0] - ref[0], path.pts[j, 1] - ref[1]))
        sp = min(2.0, max(0.3, rem))
        L.step(LoaderCommand(speed=-sp if reverse else sp, articulation=g, brake=False))
        if rem < 0.05:
            break
    return worst, L


@pytest.mark.req("AR-01")
def test_forward_tracking_error_bounded():
    cfg = load_machine()
    k = 0.85 * kappa_from_gamma(cfg.max_articulation, cfg.lf, cfg.lr)
    p = bezier((36, -2), -0.97, (40, -11.65), -math.pi / 2, k, reverse=False)
    assert p.feasible
    worst, L = track(p, (36, -2, -0.97), False, cfg)
    assert worst < 0.25
    assert math.hypot(L.state.x - 40, L.state.y + 11.65) < 0.3


@pytest.mark.req("AR-01")
def test_reverse_arc_tracking_error_bounded():
    cfg = load_machine()
    p, end = reverse_arc((46, -1.5, 0.0), -1.0, cfg.lf + cfg.lr)
    worst, L = track(p, (46, -1.5, 0.0), True, cfg)
    assert worst < 0.45
    assert math.hypot(L.state.x - end[0], L.state.y - end[1]) < 0.5


@pytest.mark.req("AR-02")
def test_ekf_bounded_through_gnss_outage_with_landmarks():
    rng = np.random.default_rng(0)
    lms = [(0, -30), (30, -30), (60, -30), (68, 9.5), (30, 28), (0, 28), (-3, 0)]
    ekf = EKF(landmarks=lms)
    ekf.init(10, 0, 0.3)
    x, y, th = 10.0, 0.0, 0.3
    errs = []
    for k in range(int(60 / 0.05)):        # 60 s, no GNSS at all
        v, w = 2.0, 0.05 * math.sin(k * 0.01)
        th += w * 0.05
        x += v * math.cos(th) * 0.05
        y += v * math.sin(th) * 0.05
        ekf.predict(v * 1.01 + rng.normal(0, 0.02), w + 0.002 + rng.normal(0, 0.003), 0.05)
        if k % 4 == 0:
            for i, (lx, ly) in enumerate(lms):
                d = math.hypot(lx - x, ly - y)
                if d < 60:
                    b = math.atan2(ly - y, lx - x) - th
                    ekf.update_landmark(i, d + rng.normal(0, 0.03),
                                        math.atan2(math.sin(b), math.cos(b)) + rng.normal(0, 0.003))
        errs.append(math.hypot(ekf.x[0] - x, ekf.x[1] - y))
    assert max(errs) < 0.3 and ekf.sigma_xy < 0.2


def test_ekf_drifts_without_any_aiding():
    ekf = EKF()
    ekf.init(0, 0, 0)
    for _ in range(1200):
        ekf.predict(2.0, 0.0, 0.05)
    assert ekf.sigma_xy > 0.6, "dead reckoning alone must report growing uncertainty"
