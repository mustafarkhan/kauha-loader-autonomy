"""Software-in-the-loop over a real CAN API (python-can virtual bus).

The same frames the autonomy computer sends in the twin are sent through
python-can between two bus endpoints; the gateway runs on the receiving
side. On the bench the 'virtual' interface is swapped for socketcan/PCAN
and the gateway for the real drive-by-wire unit (docs/hardware/hil_bench.md).
"""
from __future__ import annotations

import can
import pytest

from kauha.can.codec import Codec
from kauha.can.gateway import MachineGateway


@pytest.fixture
def buses():
    a = can.Bus(interface="virtual", channel="kauha_hil", receive_own_messages=False)
    b = can.Bus(interface="virtual", channel="kauha_hil", receive_own_messages=False)
    yield a, b
    a.shutdown()
    b.shutdown()


def send(bus, codec, name, sig):
    fid, data = codec.encode(name, sig)
    bus.send(can.Message(arbitration_id=fid, data=data, is_extended_id=True))


def pump(bus, gw, t):
    while (msg := bus.recv(timeout=0.01)) is not None:
        gw.receive(t, msg.arbitration_id, bytes(msg.data))


def test_commands_flow_and_timeout(buses):
    autonomy, machine = buses
    tx, gw = Codec(), MachineGateway()
    for k in range(10):
        t = k * 0.05
        send(autonomy, tx, "DriveCmd", {"SpeedReq": 1.5, "Mode": 1})
        send(autonomy, tx, "ImplementCmd", {"BucketRate": -0.4})
        send(autonomy, tx, "SafetyStatus", {"State": 0, "SpeedLimit": 3.0})
        pump(machine, gw, t)
        cmd = gw.command(t + 0.01)
        assert not cmd.brake and cmd.speed == pytest.approx(1.5) and cmd.bucket == pytest.approx(-0.4)
    # autonomy computer goes silent: the gateway must brake within its timeout
    assert gw.command(0.45 + 0.16).brake


def test_corrupted_frame_rejected(buses):
    autonomy, machine = buses
    tx, gw = Codec(), MachineGateway()
    fid, data = tx.encode("DriveCmd", {"SpeedReq": 1.0, "Mode": 1})
    bad = bytearray(data)
    bad[1] ^= 0x40
    autonomy.send(can.Message(arbitration_id=fid, data=bytes(bad), is_extended_id=True))
    pump(machine, gw, 0.0)
    assert gw.stats.e2e_errors == 1 and gw.command(0.01).brake
