"""Load-and-carry to a hopper (phase 2): a short nominal run of the full stack."""
from __future__ import annotations

import pytest

from kauha.demo.shift import ShiftSim


@pytest.fixture(scope="module")
def run():
    sim = ShiftSim("scripted", minutes=4.0, seed=2, task="carry")
    return sim.run(), sim


@pytest.mark.req("AR-05")
def test_completes_load_and_carry_cycles(run):
    k, sim = run
    assert sim.site.truck.kind == "hopper"
    assert k.cycles >= 2 and k.tonnes > 8
    assert 0.8 < k.fill_factor_mean < 1.15
    assert k.interventions == 0


@pytest.mark.req("AR-05")
def test_hopper_drains_and_never_leaves(run):
    _, sim = run
    tr = sim.site.truck
    assert tr.present and sim.site.trucks_completed == 0
    assert tr.payload_t < sim.site.tonnes_loaded, "the feeder drew material out"
    assert tr.spill_t == 0.0
