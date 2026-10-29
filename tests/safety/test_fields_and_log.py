from __future__ import annotations

import pytest

from kauha.safety.fields import BANDS, SCANNER_PROTECTIVE_RANGE_M, build_field_sets, field_speed_cap, select_field_set
from kauha.safety.logger import EventLog
from kauha.safety.stopping import StopBudget, max_speed_for_field, stopping_distance


@pytest.mark.req("SR-02")
@pytest.mark.parametrize("budget", [StopBudget(), StopBudget().winter()], ids=["dry", "winter"])
def test_protective_field_covers_stopping_distance(budget):
    """Each band's field covers the stopping distance at the band's top speed, or the band
    lies above the speed cap the PLC enforces for that budget."""
    sets = build_field_sets(budget)
    cap = field_speed_cap(budget)
    for fs in sets.values():
        if fs.direction == 0 or fs.name in ("dig", "dock"):
            continue
        length = max(p[0] for p in fs.protective) - min(p[0] for p in fs.protective)
        v = min(fs.v_max, cap)
        assert length >= stopping_distance(v, budget) - 0.05, fs.name


@pytest.mark.req("SR-02")
@pytest.mark.parametrize("gamma", [-0.69, -0.3, -0.12, 0.0, 0.12, 0.3, 0.69])
@pytest.mark.parametrize("direction", [1, -1])
def test_field_covers_swept_path_for_any_articulation(gamma, direction):
    """The leading contour's path over the stopping distance stays inside the selected field."""
    import math

    from kauha.safety.fields import Geometry, kappa_of, point_in_polygon
    b = StopBudget()
    sets = build_field_sets(b)
    v = 1.4 * direction
    fs = sets[select_field_set(sets, v, gamma, False, False)]
    g = Geometry()
    x0 = g.front_contour_x if direction > 0 else g.rear_contour_x
    k = kappa_of(gamma) * direction
    d = stopping_distance(v, b) - b.margin_m
    for s_ in [d * i / 10 for i in range(1, 11)]:
        th = k * s_
        cx = x0 + (math.sin(th) / k if abs(k) > 1e-9 else s_)
        cy = (1 - math.cos(th)) / k if abs(k) > 1e-9 else 0.0
        for off in (-g.half_width, 0.0, g.half_width):
            px, py = cx - math.sin(th) * off, cy + math.cos(th) * off
            assert point_in_polygon(px, py, fs.protective), (fs.name, s_, off)


def test_speed_cap_matches_inverse():
    b = StopBudget()
    v = max_speed_for_field(SCANNER_PROTECTIVE_RANGE_M, b)
    assert stopping_distance(v, b) == pytest.approx(SCANNER_PROTECTIVE_RANGE_M, abs=1e-6)
    assert 2.4 < field_speed_cap(b.winter()) < field_speed_cap(b) < 3.5


def test_selection_by_speed_and_direction():
    sets = build_field_sets(StopBudget())
    assert sets[select_field_set(sets, 0.0, 0.0, False, False)].name == "standstill"
    assert sets[select_field_set(sets, 2.2, 0.0, False, False)].name == "fwd_2.5_S"
    assert sets[select_field_set(sets, -1.2, 0.2, False, False)].name == "rev_1.5_L1"
    assert sets[select_field_set(sets, 1.2, -0.6, False, False)].name == "fwd_1.5_R2"
    assert sets[select_field_set(sets, 0.8, 0.0, True, False)].name == "dig"
    assert sets[select_field_set(sets, -0.8, 0.0, True, False)].direction == -1
    assert all(abs(v) <= BANDS[-1] for v in BANDS)


@pytest.mark.req("SR-13")
def test_log_detects_tampering():
    log = EventLog()
    for i in range(5):
        log.append(float(i), "stop", cause="FIELD_FRONT")
    assert log.verify() is None
    log.records[2]["data"]["cause"] = "NONE"
    assert log.verify() == 2
    log2 = EventLog()
    for i in range(5):
        log2.append(float(i), "stop")
    del log2.records[1]
    assert log2.verify() == 1
