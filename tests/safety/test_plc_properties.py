"""Property-based tests: invariants of the safety PLC over random input sequences."""
from __future__ import annotations

from hypothesis import given, settings
from hypothesis import strategies as st

from kauha.safety.plc import CYCLE_S, Mode, SafetyInputs, SafetyPLC, State

speeds = st.floats(min_value=-3.5, max_value=3.5, allow_nan=False)
step = st.fixed_dictionaries({
    "mode": st.sampled_from([Mode.AUTO, Mode.TELEOP, Mode.IDLE]),
    "v": speeds,
    "dv": st.floats(min_value=-0.5, max_value=0.5, allow_nan=False),
    "articulation": st.floats(min_value=-0.7, max_value=0.7, allow_nan=False),
    "front_prot": st.booleans(), "front_warn": st.booleans(),
    "rear_prot": st.booleans(), "rear_warn": st.booleans(), "side_prot": st.booleans(),
    "remote_hb_age": st.floats(min_value=0, max_value=1.0, allow_nan=False),
    "autonomy_cmd_age": st.floats(min_value=0, max_value=0.4, allow_nan=False),
    "remote_estop": st.booleans(), "machine_estop": st.booleans(), "reset_req": st.booleans(),
    "dig_req": st.booleans(), "dock_req": st.booleans(),
    "in_dig_zone": st.booleans(), "in_dock_zone": st.booleans(),
    "in_operating_area": st.booleans(), "position_valid": st.booleans(),
})


def to_inputs(t, d):
    return SafetyInputs(t=t, mode=d["mode"], speed_a=d["v"] + d["dv"], speed_b=d["v"],
                        articulation=d["articulation"], front_prot=d["front_prot"], front_warn=d["front_warn"],
                        rear_prot=d["rear_prot"], rear_warn=d["rear_warn"], side_prot=d["side_prot"],
                        remote_hb_age=d["remote_hb_age"], autonomy_cmd_age=d["autonomy_cmd_age"],
                        remote_estop=d["remote_estop"], machine_estop=d["machine_estop"],
                        reset_req=d["reset_req"], dig_req=d["dig_req"], dock_req=d["dock_req"],
                        in_dig_zone=d["in_dig_zone"], in_dock_zone=d["in_dock_zone"],
                        in_operating_area=d["in_operating_area"], position_valid=d["position_valid"])


@settings(max_examples=300, deadline=None)
@given(st.lists(step, min_size=1, max_size=60))
def test_stopped_state_always_cuts_outputs(seq):
    plc = SafetyPLC()
    for i, d in enumerate(seq):
        out = plc.step(to_inputs(i * CYCLE_S, d))
        if out.state >= State.PROTECTIVE_STOP:
            assert not out.brake_release and not out.implement_enable and out.speed_limit == 0.0
        else:
            assert out.brake_release


@settings(max_examples=300, deadline=None)
@given(st.lists(step, min_size=1, max_size=60))
def test_estop_always_results_in_emergency_stop(seq):
    plc = SafetyPLC()
    for i, d in enumerate(seq):
        out = plc.step(to_inputs(i * CYCLE_S, d))
        if d["machine_estop"] or d["remote_estop"]:
            assert out.state == State.EMERGENCY_STOP


@settings(max_examples=300, deadline=None)
@given(st.lists(step, min_size=1, max_size=60))
def test_speed_limit_never_exceeds_field_cap(seq):
    plc = SafetyPLC()
    for i, d in enumerate(seq):
        out = plc.step(to_inputs(i * CYCLE_S, d))
        assert out.speed_limit <= plc.field_cap + 1e-9


@settings(max_examples=300, deadline=None)
@given(st.lists(step, min_size=1, max_size=60))
def test_moving_protective_field_violation_never_runs(seq):
    """A violated protective field of the active direction never leaves the machine running
    (except in supervised teleop recovery, SR-12)."""
    plc = SafetyPLC()
    for i, d in enumerate(seq):
        out = plc.step(to_inputs(i * CYCLE_S, d))
        fs = plc.sets[out.field_set]
        if plc.recovery or d["mode"] == Mode.TELEOP:
            continue
        if d["front_prot"] and fs.direction >= 0 and fs.protective:
            assert out.state >= State.PROTECTIVE_STOP


@settings(max_examples=100, deadline=None)
@given(st.lists(step, min_size=1, max_size=40))
def test_deterministic(seq):
    a, b = SafetyPLC(), SafetyPLC()
    for i, d in enumerate(seq):
        oa, ob = a.step(to_inputs(i * CYCLE_S, d)), b.step(to_inputs(i * CYCLE_S, d))
        assert oa == ob
    assert [r["hash"] for r in a.log.records] == [r["hash"] for r in b.log.records]
