"""Fault injection with real processes: kill a unit and check the machine stops.

Each scenario runs the machine, the safety PLC, the autonomy computer and the
remote station as separate processes on a shared CAN bus (UDP multicast) and a
hardwired-I/O link, in real time (about 10 s each).
"""
from __future__ import annotations

import pytest

from kauha.sil.run import run

pytestmark = pytest.mark.slow


@pytest.mark.req("SR-02")
def test_person_in_path_stops_machine_across_processes(tmp_path):
    r = run("person", tmp_path)
    assert r["v_max"] > 2.0 and r["v_end"] == 0.0
    assert r["min_gap_m"] > 0.5
    assert any(c == "FIELD_FRONT" for t, _, c in r["stops"] if t > 2.0)


@pytest.mark.req("SR-06")
def test_killing_the_autonomy_process_stops_the_machine(tmp_path):
    r = run("kill_autonomy", tmp_path)
    assert r["brake_after_s"] is not None and r["brake_after_s"] <= 0.3
    assert r["standstill_after_s"] <= 1.6
    assert any(c == "WATCHDOG" for _, _, c in r["stops"])


@pytest.mark.req("SR-05")
def test_losing_the_remote_station_stops_the_machine(tmp_path):
    r = run("kill_remote", tmp_path)
    assert 0.4 <= r["brake_after_s"] <= 0.7
    assert any(c == "REMOTE_LINK" for _, _, c in r["stops"])


@pytest.mark.req("SR-15")
def test_killing_the_safety_plc_drops_the_relays(tmp_path):
    r = run("kill_plc", tmp_path)
    assert r["brake_after_s"] is not None and r["brake_after_s"] <= 0.2
    assert r["v_end"] == 0.0


def test_nominal_run_reaches_speed(tmp_path):
    r = run("nominal", tmp_path)
    assert r["v_max"] >= 1.95 and r["overruns_machine"] < 10
