"""Every stopping field lies inside the rated coverage of a safety sensor (kauha/safety/coverage.py)."""
from __future__ import annotations

import math

import numpy as np
import pytest

from kauha.safety.coverage import LAYOUT, reach, scanner_only_speed_cap, sensors, uncovered
from kauha.safety.fields import build_field_sets
from kauha.safety.stopping import StopBudget
from kauha.sim.sensors import FRONT_MOUNTS, REAR_MOUNTS, SafetyScanners


@pytest.mark.req("SR-04")
def test_all_stopping_fields_are_inside_rated_coverage_dry_and_winter():
    for b in (StopBudget(), StopBudget().winter()):
        assert uncovered(build_field_sets(b)) == []


@pytest.mark.req("SR-04")
def test_scanners_alone_do_not_cover_the_fields():
    sets = build_field_sets(StopBudget())
    far = reach(sets)
    assert far["scanner"] > 4.0 and far["radar"] <= 9.0
    miss = {name for _, name, _, _ in uncovered(sets, kinds=("scanner",))}
    assert miss == {"protective"}                    # side, contact and under-bucket fields need no radar
    assert scanner_only_speed_cap(StopBudget()) < 0.5


def test_layout_matches_the_twin_mounts_and_the_radar_unit_limit():
    twin = {m.name: (m.body, m.x, m.y, m.heading) for m in FRONT_MOUNTS + REAR_MOUNTS}
    rated = {s.name: (s.body, s.x, s.y, s.heading) for s in LAYOUT if s.kind == "scanner"}
    assert twin == rated
    assert len([s for s in LAYOUT if s.kind == "radar"]) == 6        # one control unit takes six sensors
    assert len(sensors("front")) == len(sensors("rear")) == 5


@pytest.mark.req("SR-04")
def test_twin_ignores_returns_outside_rated_coverage():
    """A wide object 12 m ahead is inside the scanners' 15 m model range but outside every rated
    zone: it must not count toward a stopping field even if the field were drawn that far."""
    sets = build_field_sets(StopBudget())
    fs = sets[max(k for k, v in sets.items() if v.name.startswith("fwd") and v.name.endswith("_S"))]
    long_field = [(5.1, -1.9), (14.0, -1.9), (14.0, 1.9), (5.1, 1.9)]
    stretched = type(fs)(**{**fs.__dict__, "protective": long_field})
    poses = {"front": (0.0, 0.0, 0.0), "rear": (-3.2, 0.0, math.pi)}

    def stops(x):
        sc = SafetyScanners(np.random.default_rng(0))
        wall = np.array([[x, -1.5, x, 1.5]])
        return [sc.scan(poses, stretched, wall, np.zeros((0, 3))).front_prot for _ in range(2)][-1]

    assert stops(8.0) and not stops(12.0)
