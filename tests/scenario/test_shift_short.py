"""Short end-to-end run of the full stack: autonomy, CAN, gateway, PLC, scanners, twin."""
from __future__ import annotations

import pytest

from kauha.demo.shift import ShiftSim


@pytest.fixture(scope="module")
def kpis():
    sim = ShiftSim("scripted", minutes=4.5, seed=3)
    return sim.run(), sim


@pytest.mark.req("AR-03")
def test_completes_loading_cycles(kpis):
    k, _ = kpis
    assert k.cycles >= 3 and k.tonnes > 12
    assert 0.8 < k.fill_factor_mean < 1.15


@pytest.mark.req("SR-13")
def test_safety_log_intact(kpis):
    k, sim = kpis
    assert k.log_chain_ok
    assert all(e["field"] for e in sim._stop_events)


def test_no_bus_integrity_errors(kpis):
    k, _ = kpis
    assert k.e2e_errors == 0


def test_localisation_accuracy(kpis):
    k, _ = kpis
    assert k.loc_rms_m < 0.1
