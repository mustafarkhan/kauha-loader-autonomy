"""The PLC's CAN and I/O glue (FB_SafetyNode) on the 1:8 testbed profile.

Runs the matiec build of plc/codesys/src in closed loop with frames encoded by
the Python codec (the same DBC the ROS 2 bridge and the gateway firmware use):
boot into a safe stop, reset, run, then lose each input in turn.
"""
from __future__ import annotations

import subprocess

import pytest

from kauha.can.codec import Codec
from kauha.can.e2e import E2EReceiver
from kauha.safety.plc import Cause
from kauha.scale.profile import scale_profile
from kauha.verify import st_equivalence as eq

pytestmark = pytest.mark.skipif(eq.find_iec2c() is None, reason="matiec iec2c not installed (set KAUHA_IEC2C)")

RUN, PSTOP, ESTOP = 0, 2, 3


@pytest.fixture(scope="module")
def exe(tmp_path_factory):
    return eq.build(tmp_path_factory.mktemp("node"))


class Node:
    """Drives the harness one 25 ms cycle at a time and plays every other node on the bus."""

    def __init__(self, exe, profile=None):
        self.p = profile or scale_profile("evaluator")
        self.proc = subprocess.Popen([str(exe), "node"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                                     bufsize=1)
        self.proc.stdin.write(" ".join(eq._fmt(x) for x in eq.config_values(self.p)) + "\n")
        self.codec = Codec()
        self.rx_status = E2EReceiver(self.codec.data_id("SafetyStatus"), 0.1)
        self.t = 0.0
        self.k = 0
        self.last_set = 0
        self.status_ok = 0
        # what the other nodes send
        self.mode, self.remote = 1, True
        self.estop_remote = False
        self.reset = False
        self.speed = 0.0
        self.sensors, self.corrupt_sensors = True, False
        self.evaluator, self.echo_offset = True, 0
        self.zone = True
        self.estop_chain_ok = True
        self.fields: dict = {}

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=10)

    def frames(self):
        out = []
        enc = self.codec.encode
        if self.remote and self.k % 2 == 0:
            out.append(enc("RemoteHeartbeat", dict(EStop=int(self.estop_remote), ResetReq=int(self.reset),
                                                  ModeReq=self.mode)))
        if self.k % 2 == 0:
            out.append(enc("DriveCmd", dict(SpeedReq=self.speed, Mode=self.mode)))
        out.append(enc("MachineState1", dict(Speed=self.speed)))
        if self.sensors:
            fid, data = enc("SafeSensors", dict(SpeedB=self.speed, Articulation=0.0, BoomAngle=-0.2, BoomRaised=0,
                                                SensorOk=1))
            if self.corrupt_sensors:
                data = bytes([data[0] ^ 0x01]) + data[1:]
            out.append((fid, data))
        if self.evaluator:
            bits = {"ScannersOk": 1, **self.fields}
            out.append(enc("ScannerStatus", dict(EvaluatedSet=(self.last_set + self.echo_offset) % 256, **bits)))
        if self.zone and self.k % 4 == 0:
            out.append(enc("ZoneStatus", dict(InOperatingArea=1, PositionValid=1)))
        return out

    def cycle(self):
        self.t += 0.025
        fr = self.frames()
        io = [1, 1, 1, 1, 1, 1, 1]
        line = [self.t.hex(), int(self.estop_chain_ok), 0, *io, len(fr)]
        for fid, data in fr:
            line += [fid & 0x1FFFFFFF, *data]
        self.proc.stdin.write(" ".join(map(str, line)) + "\n")
        parts = self.proc.stdout.readline().split()
        self.k += 1
        state, cause, limit, fset, drive, impl = (int(parts[0]), int(parts[1]), float.fromhex(parts[2]),
                                                  int(parts[3]), parts[4] == "1", parts[5] == "1")
        tx_id = int(parts[7])
        tx = bytes(int(x) for x in parts[10:18])
        name, sig = self.codec.decode(tx_id | 0x80000000 if tx_id < 0x80000000 else tx_id, tx)
        assert name == "SafetyStatus"
        if self.rx_status.check(tx, self.t).value in ("ok", "ok_some_lost"):
            self.status_ok += 1
        assert sig["State"] == state and sig["ActiveFieldSet"] == fset
        self.last_set = fset
        return dict(state=state, cause=Cause(cause), limit=limit, drive=drive, impl=impl,
                    sensor_fault=parts[8] == "1", scanner_fault=parts[9] == "1")

    def run(self, seconds):
        r = None
        for _ in range(round(seconds / 0.025)):
            r = self.cycle()
        return r

    def reset_pulse(self):
        self.reset = True
        self.run(0.1)
        self.reset = False
        return self.run(0.2)


@pytest.fixture
def node(exe):
    n = Node(exe)
    yield n
    n.close()


def test_boots_stopped_then_runs_after_reset(node):
    r = node.cycle()
    assert r["state"] >= PSTOP and not r["drive"]           # fail-safe at power-up
    r = node.run(1.0)
    assert r["state"] >= PSTOP
    r = node.reset_pulse()
    assert r["state"] == RUN and r["drive"] and r["impl"]
    assert 0 < r["limit"] <= node.p.table.speed_cap
    assert node.status_ok >= node.k - 2                      # every SafetyStatus passes the receiver's E2E check


def ready(node):
    node.run(0.5)
    r = node.reset_pulse()
    assert r["state"] == RUN
    node.speed = 0.2
    return node.run(0.5)


def test_remote_link_loss_stops(node):
    assert ready(node)["state"] == RUN
    node.remote = False
    r = node.run(node.p.params.remote_timeout_s + 0.1)
    assert r["state"] == PSTOP and Cause.REMOTE_LINK in r["cause"] and not r["drive"]


def test_corrupted_sensor_frames_are_a_fault(node):
    ready(node)
    node.corrupt_sensors = True
    r = node.run(0.2)
    assert r["sensor_fault"] and r["state"] == PSTOP and Cause.SCANNER_FAULT in r["cause"]


def test_evaluator_on_wrong_field_set_is_a_fault(node):
    ready(node)
    node.echo_offset = 7
    r = node.run(0.1)
    assert r["scanner_fault"] and r["state"] == PSTOP


def test_machine_estop_chain(node):
    ready(node)
    node.estop_chain_ok = False
    r = node.cycle()
    assert r["state"] == ESTOP and Cause.ESTOP_MACHINE in r["cause"] and not r["drive"] and not r["impl"]
    node.estop_chain_ok = True
    node.speed = 0.0
    node.run(0.2)
    assert node.reset_pulse()["state"] == RUN


def test_front_field_stops_and_zone_loss_stops_in_auto(node):
    ready(node)
    node.fields = {"FrontProt": 1}
    r = node.cycle()
    assert r["state"] == PSTOP and Cause.FIELD_FRONT in r["cause"]
    node.fields = {}
    node.speed = 0.0
    node.run(0.3)
    node.reset_pulse()
    node.zone = False
    r = node.run(0.5 + node.p.params.position_invalid_s + 0.2)    # zone frame timeout, then SR-07
    assert r["state"] == PSTOP and Cause.POSITION_INVALID in r["cause"]
