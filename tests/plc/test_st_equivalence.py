"""The CODESYS safety program behaves exactly like the proved reference (SR-01..SR-16).

Compiles plc/codesys/src with the matiec IEC 61131-3 compiler and compares it
cycle by cycle with kauha/safety/core.py on every profile (full size and the
1:8 testbed, automatic and supervisor-only restart). Skipped when matiec is not
installed; CI builds it.
"""
from __future__ import annotations

import pytest

from kauha.safety.plc import Cause
from kauha.verify import st_equivalence as eq

pytestmark = pytest.mark.skipif(eq.find_iec2c() is None, reason="matiec iec2c not installed (set KAUHA_IEC2C)")


@pytest.fixture(scope="module")
def exe(tmp_path_factory):
    return eq.build(tmp_path_factory.mktemp("st"))


@pytest.mark.parametrize("profile", eq.profiles(), ids=lambda p: p.name)
def test_st_matches_reference(exe, profile):
    rep = eq.compare(exe, profile, 30_000, seed=7)
    assert rep.mismatches == 0, rep.first
    assert {2, 3} <= set(rep.states_seen)
    assert rep.events["recovery"] > 0 and rep.events["reset"] > 0


def test_every_cause_is_exercised(exe):
    seen = 0
    for k, p in enumerate(eq.profiles()[:3]):
        seen |= eq.compare(exe, p, 30_000, seed=11 + k).causes_seen
    missing = [c.name for c in Cause if c and not seen & c]
    assert not missing, f"the generator never latched {missing}"


@pytest.mark.parametrize("mutation", [
    ("c := HAS(rec, FIELD_FRONT) AND (v > cfg.standstill);", "c := HAS(rec, FIELD_FRONT) AND (v >= cfg.standstill);"),
    ("over := ABS(v) > allowed + cfg.overspeed_tol;", "over := ABS(v) > allowed;"),
    ("ELSIF a_v <= cfg.band1 THEN band := 1;", "ELSIF a_v < cfg.band1 THEN band := 1;"),
    ("posinv_on := SEL(is_auto, m_posinv_on, NOT inp.position_valid);",
     "posinv_on := SEL(is_auto, m_posinv_on, inp.position_valid);"),
], ids=["recovery-edge", "overspeed-tolerance", "band-boundary", "posinv-memory"])
def test_comparison_catches_a_seeded_fault(tmp_path, mutation):
    exe = eq.build(tmp_path, mutation=mutation)
    reps = [eq.compare(exe, p, 30_000, seed=7) for p in eq.profiles()[:3]]
    assert sum(r.mismatches for r in reps) > 0


def test_bus_functions_match_python_codec(exe):
    assert eq.check_crc(exe, 5_000, 3) == 0
    assert eq.check_e2e(exe, 5_000, 3) == 0
    assert eq.check_status_frame(exe, 5_000, 3) == 0
