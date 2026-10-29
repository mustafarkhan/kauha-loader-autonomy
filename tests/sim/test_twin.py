from __future__ import annotations

import math

import numpy as np
import pytest

from kauha.config import load_machine
from kauha.sim.loader import Loader, LoaderCommand, LoaderState
from kauha.sim.sensors import raycast
from kauha.sim.soil import BucketPileInteraction, Pile, bucket_capacity
from kauha.sim.world import load_site


def test_articulated_turning_radius_matches_kinematics():
    cfg = load_machine()
    g = 0.5
    L = Loader(cfg, LoaderState(gamma=g, v=1.0), dt=0.01)
    pts = []
    for _ in range(3000):
        L.step(LoaderCommand(speed=1.0, articulation=g, brake=False))
        pts.append((L.state.x, L.state.y))
    P = np.array(pts[500:])
    # algebraic (Kasa) circle fit
    A = np.column_stack([2 * P[:, 0], 2 * P[:, 1], np.ones(len(P))])
    b = (P ** 2).sum(axis=1)
    cx, cy, c0 = np.linalg.lstsq(A, b, rcond=None)[0]
    R = math.sqrt(c0 + cx * cx + cy * cy)
    expected = (cfg.lf * math.cos(g) + cfg.lr) / math.sin(g)
    assert R == pytest.approx(expected, rel=0.03)


def test_actuator_latency_and_braking():
    cfg = load_machine()
    L = Loader(cfg, LoaderState(v=2.0), dt=0.02)
    L.step(LoaderCommand(brake=True))
    assert L.state.v == pytest.approx(2.0, abs=0.06), "brake must not act before the actuator latency"
    for _ in range(200):
        L.step(LoaderCommand(brake=True))
    assert L.state.v == pytest.approx(0.0, abs=1e-6)


def test_raycast_hits_segment_and_circle():
    segs = np.array([(5, -5, 5, 5)], dtype=float)
    circ = np.array([(3, 0, 0.5)], dtype=float)
    r = raycast(0, 0, np.array([0.0, math.pi / 2, math.atan2(4, 5)]), 20, segs, circ)
    assert r[0] == pytest.approx(2.5) and r[1] == 20 and r[2] == pytest.approx(math.hypot(5, 4), rel=1e-6)


def test_soil_resistance_grows_with_depth_and_stiffness():
    soft = BucketPileInteraction(Pile("s", 1.6, 1.0, 33), 2.7, 3.0)
    hard = BucketPileInteraction(Pile("h", 1.6, 2.1, 42), 2.7, 3.0)
    for it in (soft, hard):
        it.penetration = 1.0
    rs = soft.step(dx_forward=0.01, tip_z=0, tip_dz=0, attitude=0, attitude_rate=0, load_m3=0, dt=0.05)
    rh = hard.step(dx_forward=0.01, tip_z=0, tip_dz=0, attitude=0, attitude_rate=0, load_m3=0, dt=0.05)
    assert rh.resist_force_n > 2 * rs.resist_force_n > 0
    assert bucket_capacity(3.0, 0.6) > 3.0 > bucket_capacity(3.0, 0.0)


def test_pile_face_recedes():
    cfg = load_machine()
    site = load_site(cfg)
    p = site.piles["A"]
    s = p.slots[0]
    x0 = s.toe_x
    p.remove(s, 3.0)
    assert s.toe_x > x0
