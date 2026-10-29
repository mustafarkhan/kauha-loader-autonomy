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


@pytest.mark.req("AR-04")
def test_field_guard_slows_before_a_mapped_object_enters_the_field():
    from kauha.autonomy.field_guard import FieldGuard
    from kauha.autonomy.paths import straight
    g = FieldGuard()
    path = straight((0.0, 0.0), 0.0, 40.0, reverse=False)
    g.set_map([(30.0, -5.0, 30.0, 5.0)], [])          # a wall across the path at x = 30
    caps = [g.speed_cap(path.pts, path.heading, path.s, path.nearest(x, 0.0), False, (0.0,))
            for x in (5.0, 18.0, 21.0, 22.5, 24.0)]
    assert caps[0] >= 2.8, "far away: full speed"
    assert all(a >= b for a, b in zip(caps, caps[1:])), "cap falls as the wall gets closer"
    assert caps[-1] == 0.0, "no band is clear once the wall is inside the slowest field"
    g.set_map([], [])
    assert g.speed_cap(path.pts, path.heading, path.s, 0, False, (0.0,)) >= 2.8


@pytest.mark.req("AR-04")
def test_field_guard_uses_the_rear_fields_when_reversing():
    from kauha.autonomy.field_guard import FieldGuard
    from kauha.autonomy.paths import straight
    g = FieldGuard()
    path = straight((0.0, 0.0), math.pi, 20.0, reverse=True)   # rear axle path heading -x
    g.set_map([(-6.0, -5.0, -6.0, 5.0)], [])
    cap = g.speed_cap(path.pts, path.heading, path.s, 0, True, (0.0,))
    assert 0.0 < cap < 2.0, "a wall 6 m behind the rear axle limits reversing speed"


@pytest.mark.req("AR-02")
def test_ekf_recovers_from_overconfident_error_using_posts():
    """A 2 m error with a tight covariance (e.g. after slip) is recovered without GNSS."""
    lms = [(0, -30), (30, -30), (60, -30), (68, 9.5), (30, 28), (0, 28), (-3, 0)]
    ekf = EKF(landmarks=lms)
    ekf.init(12.0, 1.5, 0.0, sigma=0.02)            # truth is (10, 0, 0)
    for _ in range(40):
        for i, (lx, ly) in enumerate(lms):
            ekf.update_landmark(i, math.hypot(lx - 10, ly), math.atan2(ly, lx - 10))
    assert math.hypot(ekf.x[0] - 10, ekf.x[1]) < 0.2 and ekf.inflations > 0
