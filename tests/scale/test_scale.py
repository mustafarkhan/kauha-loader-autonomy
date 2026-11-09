"""1:8 testbed: scaling, linkages, field evaluator and CAD checks."""
from __future__ import annotations

import json
import math

import pytest

from kauha.config import REPO_ROOT
from kauha.safety.stopping import stopping_distance
from kauha.scale import linkage
from kauha.scale.field_eval import evaluate, load_sets
from kauha.scale.profile import F, check, full_profile, scale_profile

GEN = REPO_ROOT / "hardware" / "scale" / "generated"


def test_froude_laws():
    assert F.length(8.0) == 1.0
    assert F.speed(3.0) == pytest.approx(3.0 / math.sqrt(8))
    assert F.mass(15500) == pytest.approx(30.27, abs=0.01)
    assert F.torque(8 ** 4) == 1.0


@pytest.mark.parametrize("scanner", ["nanoscan3", "evaluator"])
def test_scaled_profiles_keep_the_safety_rules(scanner):
    p = scale_profile(scanner)
    assert check(p) == []
    t = p.table
    # every band's field stops the machine from the speed it protects (SR-04)
    for v in t.v_protected:
        assert stopping_distance(v, p.budget) <= p.design.protective_range + 1e-9
    assert t.speed_cap < t.field_cap and p.params.side_recovery_speed <= p.params.recovery_speed
    assert len(p.field_sets()) == len(full_profile().field_sets())


def test_generated_profile_and_fields_are_current():
    rep = json.loads((GEN / "scale_profile.json").read_text())
    p = scale_profile("evaluator")
    assert rep["profiles"][p.name]["field_table"]["speed_cap"] == pytest.approx(p.table.speed_cap)
    assert set(load_sets()) == set(p.field_sets())


def test_linkage_design_is_current_and_checked():
    stored = json.loads((GEN / "linkage.json").read_text())
    assert stored["checks"] == "pass"
    fresh = linkage.design(stored["seed"])
    for name, a in fresh["actuators"].items():
        assert a["lengths"] == pytest.approx(stored["actuators"][name]["lengths"], abs=1e-9), name
    assert linkage.check(fresh) == []


def test_field_evaluator_sees_a_post_ahead():
    sets = load_sets()
    p = scale_profile("evaluator")
    sid = 1 + 1 * 5 + 2          # forward, second band, straight
    fs = sets[sid]
    xs = [x for x, _ in fs["protective"]]
    target = (min(xs) + max(xs)) / 2
    pts = [(target, -0.01), (target, 0.0), (target, 0.01)]
    r = evaluate(sets, sid, pts, [], boom_raised=False)
    assert r["FrontProt"] == 1 and r["FrontWarn"] == 1 and r["RearProt"] == 0 and r["ScannersOk"] == 1
    assert evaluate(sets, sid, [], [], False)["FrontProt"] == 0
    assert evaluate(sets, 250, [], [], False)["ScannersOk"] == 0
    assert p.design.protective_range == pytest.approx(0.5)
