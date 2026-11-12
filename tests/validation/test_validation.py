"""External validation: criteria, checks and the results table."""
from __future__ import annotations

import json

import numpy as np

from kauha.can.e2e import protect
from kauha.validation import checks as C
from kauha.validation.criteria import CRITERIA
from kauha.validation.report import JSON, KNOWN_FAILURES, MD, markdown, run


def test_every_criterion_has_a_source_a_target_and_a_check():
    ids = [c.id for c in CRITERIA]
    assert len(ids) == len(set(ids))
    for c in CRITERIA:
        assert c.url.startswith("https://") and c.target and c.metric
        assert c.id in C.CHECKS
        assert (c.kind == "dataset") or not c.data_env or c.id == "V-PUB-04"


def test_results_table_is_current_and_no_new_failure():
    rows = run()
    failed = {r["id"] for r in rows if r["status"] == "fail"}
    assert failed <= KNOWN_FAILURES, f"new failures: {sorted(failed - KNOWN_FAILURES)}"
    assert json.loads(JSON.read_text()) == rows, "run python -m kauha.validation.report"
    assert MD.read_text() == markdown(rows)


def test_attacks_measure_what_they_claim():
    fz = C.attack_fuzzing(n=40_000)
    assert 0 < fz["rate"] < 0.002                       # near 1/256 x 2/16: chance, not protection
    assert C.attack_masquerade(cycles=50)["gateway_released_while_plc_stopped"] == 50
    assert C.attack_suspension()["brake_after_s"] <= 0.16


def test_candump_log_replay(tmp_path):
    log = tmp_path / "a.log"
    good = protect(bytearray(8), 0x10, 1)
    log.write_text("(1.000000) can0 123#DEADBEEF00112233\n(1.010000) can0 0C0#0102030405060708\nnoise\n"
                   f"(1.020000) can0 18FF1080#{good.hex().upper()}\n")
    frames = C.parse_candump(log)
    assert [f[1] for f in frames] == [0x123, 0x0C0, 0x18FF1080]
    r = C.replay_log_on_bus(frames)
    assert r["frames"] == 3 and r["decoded_as_recorded"] is False


def test_dust_frame_reader_and_field_check(tmp_path):
    pts = np.array([[2.0, 0.0, 0.0, 9.0], [2.1, 0.1, 0.05, 9.0], [2.0, 0.0, 1.5, 9.0], [30.0, 0.0, 0.0, 1.0]],
                   dtype=np.float32)
    b = tmp_path / "sequence_000.bin"
    pts.tofile(b)
    field = [(0.5, -1.0), (4.0, -1.0), (4.0, 1.0), (0.5, 1.0)]
    p, boxes = C.read_dustx_frame(b)
    assert p.shape == (4, 4) and boxes == []
    assert C.dust_frame_false_occupied(p, boxes, field) == (True, False)       # dust-like returns, no object
    b.with_suffix(".txt").write_text("truck 3.0 2.5 6.0 2.5 0.2 0.0 0.1\n")
    p, boxes = C.read_dustx_frame(b)
    assert C.dust_frame_false_occupied(p, boxes, field) == (True, True)
