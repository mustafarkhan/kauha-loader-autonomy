from __future__ import annotations

import pytest

from kauha.can.codec import SA, Codec, j1939_id, pgn_of
from kauha.can.e2e import E2EStatus, crc8_j1850
from kauha.can.gateway import MachineGateway


def test_crc8_sae_j1850_check_value():
    assert crc8_j1850(b"123456789") == 0x4B


def test_dbc_ids_are_j1939():
    c = Codec()
    expect = {"DriveCmd": (3, 0xFF10, SA["AUTONOMY"]), "ImplementCmd": (3, 0xFF11, SA["AUTONOMY"]),
              "SafetyStatus": (2, 0xFF20, SA["SAFETY_PLC"]), "MachineState1": (3, 0xFF30, SA["MACHINE_GW"]),
              "MachineState2": (3, 0xFF31, SA["MACHINE_GW"]), "RemoteHeartbeat": (3, 0xFF40, SA["REMOTE_GW"])}
    for name, (prio, pgn, sa) in expect.items():
        m = c.db.get_message_by_name(name)
        assert m.is_extended_frame and m.frame_id == j1939_id(prio, pgn, sa)
        assert pgn_of(m.frame_id) == pgn
    data_ids = [c.data_id(n) for n in expect]
    assert len(set(data_ids)) == len(data_ids), "E2E data IDs must be unique"


def test_roundtrip_and_counters():
    c = Codec()
    rx = c.receiver("DriveCmd", 0.15)
    for i in range(20):
        fid, data = c.encode("DriveCmd", {"SpeedReq": 1.234, "ArticulationReq": -0.2, "Mode": 1})
        assert rx.check(data, i * 0.05) == E2EStatus.OK
        name, sig = c.decode(fid, data)
        assert sig["SpeedReq"] == pytest.approx(1.234) and sig["AliveCounter"] == i % 16


def test_e2e_detects_corruption_repeat_loss_and_masquerade():
    c = Codec()
    rx = c.receiver("DriveCmd", 0.15)
    _, d0 = c.encode("DriveCmd", {"SpeedReq": 1.0, "Mode": 1})
    assert rx.check(d0, 0.0) == E2EStatus.OK
    bad = bytearray(d0)
    bad[0] ^= 0x01
    assert rx.check(bytes(bad), 0.05) == E2EStatus.CRC_ERROR
    assert rx.check(d0, 0.05) == E2EStatus.REPEATED
    for _ in range(4):
        _, d = c.encode("DriveCmd", {"SpeedReq": 1.0, "Mode": 1})
    assert rx.check(d, 0.1) == E2EStatus.WRONG_SEQUENCE
    # an ImplementCmd payload replayed under the DriveCmd ID fails (data ID in CRC)
    _, imp = c.encode("ImplementCmd", {"BoomRate": 0.5})
    assert rx.check(imp, 0.15) == E2EStatus.CRC_ERROR


def _feed(gw, c, t, speed=2.0, limit=3.0, state=0):
    for name, sig in (("DriveCmd", {"SpeedReq": speed, "Mode": 1}), ("ImplementCmd", {"BoomRate": 0.5}),
                      ("SafetyStatus", {"State": state, "SpeedLimit": limit})):
        fid, data = c.encode(name, sig)
        gw.receive(t, fid, data)


def test_gateway_clamps_to_safety_limit_and_times_out():
    c, gw = Codec(), MachineGateway()
    _feed(gw, c, 0.0, speed=2.5, limit=1.0)
    cmd = gw.command(0.01)
    assert not cmd.brake and cmd.speed == pytest.approx(1.0) and cmd.boom == pytest.approx(0.5)
    assert gw.command(0.2).brake, "stale commands must brake"


def test_gateway_brakes_on_safety_stop():
    c, gw = Codec(), MachineGateway()
    _feed(gw, c, 0.0, state=2)
    cmd = gw.command(0.01)
    assert cmd.brake and cmd.boom == 0.0
