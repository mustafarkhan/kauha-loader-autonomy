"""External validation: criteria, checks and the results table."""
from __future__ import annotations

import json

import numpy as np
import pytest

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
    # at a receiver on the safety segment the CRC alone lets chance hits and a computed CRC through
    assert 0 < C.direct_fuzzing(n=40_000)["rate"] < 0.002
    assert C.direct_masquerade(cycles=50)["gateway_released_while_plc_stopped"] == 50
    # from the open segment, through the guard, nothing forged gets in and genuine traffic mostly does
    fz = C.attack_fuzzing(n=20_000)
    assert fz["accepted"] == 0 and fz["intrusions"] > 1000 and 0.9 < fz["genuine_delivered"] < 1.0
    mq = C.attack_masquerade(cycles=50)
    assert mq["gateway_released_while_plc_stopped"] == 0 and mq["forged_heartbeats_accepted"] == 0
    assert C.attack_replay(n=2000)["accepted"] == 0
    assert C.attack_suspension()["brake_after_s"] <= 0.16


def test_candump_log_replay(tmp_path):
    log = tmp_path / "a.log"
    good = protect(bytearray(8), 0x10, 1)
    log.write_text("(1.000000) can0 123#DEADBEEF00112233\n(1.010000) can0 0C0#0102030405060708\nnoise\n"
                   f"(1.020000) can0 18FF1080#{good.hex().upper()}\n")
    frames = C.parse_candump(log)
    assert [f[1] for f in frames] == [0x123, 0x0C0, 0x18FF1080]
    r = C.replay_log_on_bus(frames)
    assert r["frames"] == 3 and r["decoded_as_recorded"] is False and r["forwarded_by_guard"] == 0


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


def test_boreas_frames_and_field_rate(tmp_path):
    lidar = tmp_path / "boreas-2021-01-26-10-59" / "lidar"
    lidar.mkdir(parents=True)
    field = [(2.5, -1.9), (6.5, -1.9), (6.5, 1.9), (2.5, 1.9)]
    # sensor 2.0 m up: the 0.5 m slice is z = -1.5. One frame with returns in the field, one without.
    hit = np.array([[4.0, 0.0, -1.5, 5, 1, 0.0], [4.1, 0.2, -1.45, 5, 2, 0.0], [4.0, 0.0, 0.5, 5, 3, 0.0]], np.float32)
    miss = np.array([[4.0, 0.0, 0.5, 5, 1, 0.0], [30.0, 0.0, -1.5, 5, 2, 0.0]], np.float32)
    hit.tofile(lidar / "1611676800000000.bin")
    miss.tofile(lidar / "1611676800100000.bin")
    assert C.read_boreas_frame(lidar / "1611676800000000.bin").shape == (3, 6)
    assert C.boreas_occupied_rate(lidar.parent, field, 2.0, 10) == (2, 0.5)


def test_radar_cfar_finds_a_bright_target_and_not_noise():
    rng = np.random.default_rng(0)
    img = rng.integers(5, 25, size=(200, 200)).astype(np.uint8)
    assert not C.cfar_detects(img, 100, 100)
    img[98:103, 98:103] = 180
    assert C.cfar_detects(img, 100, 100)
    assert not C.cfar_detects(img, 3, 3)                 # too close to the image edge to judge


def test_radiate_people_reads_labels(tmp_path):
    pil = pytest.importorskip("PIL.Image")
    seq = tmp_path / "fog_6_0"
    (seq / "Navtech_Cartesian").mkdir(parents=True)
    (seq / "annotations").mkdir()
    img = np.full((1152, 1152), 10, np.uint8)
    img[596:601, 596:601] = 200                           # a target about 3.9 m from the radar
    pil.fromarray(img).save(seq / "Navtech_Cartesian" / "000001.png")
    (seq / "annotations" / "annotations.json").write_text(json.dumps([
        {"id": 1, "class_name": "pedestrian", "bboxes": [{"position": [594, 594, 9, 9], "rotation": 0}]},
        {"id": 2, "class_name": "car", "bboxes": [{"position": [100, 100, 20, 40], "rotation": 0}]},
        {"id": 3, "class_name": "pedestrian", "bboxes": [{"position": [800, 800, 9, 9], "rotation": 0}]}]))
    people = C.radiate_people(seq, 10)
    assert [det for _, det in people] == [True, False]
    assert people[0][0] == pytest.approx(5.4, abs=0.2) and people[1][0] > 30
