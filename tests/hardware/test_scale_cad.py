"""1:8 testbed CAD: actuator strokes, clearances, stability, manufacturing files."""
from __future__ import annotations

import pytest

pytest.importorskip("cadquery")
from hardware.scale.cad import scale_cad as S  # noqa: E402


def test_actuators_stay_within_their_strokes():
    assert S.actuator_length_check() == []


@pytest.mark.parametrize("pose", [S.CARRY, dict(gamma=0.69, boom=-0.62, bucket=0.0, osc=0.0),
                                  dict(gamma=-0.69, boom=0.85, bucket=-0.7, osc=0.0),
                                  dict(gamma=0.0, boom=0.95, bucket=0.5, osc=S.REAR_OSC)],
                         ids=["carry", "dig-full-turn", "dump-full-turn", "raised-oscillated"])
def test_no_interference(pose):
    assert S.interference(pose) == []


def test_ground_clearance_and_stability():
    assert S.ground_check(S.CARRY) == []
    st = S.tipping()
    sand = S.bucket_volume_l() * 1.6
    assert st["rated_operating_load_kg"] >= sand          # ISO 14397-1: ROL <= 50 % of full-turn tipping load
    assert st["tipping_load_full_turn_kg"] < st["tipping_load_straight_kg"]


def test_flat_parts_have_drawings_and_dxfs():
    groups = S.plate_groups(S.build(S.CARRY))
    assert len(groups) >= 10
    fp = S.shell_flat_pattern()
    assert len(fp["bend_angles_deg"]) == len(S.BUCKET_PROFILE) - 2
    assert 300 < fp["flat_length_mm"] < 360
