"""Firmware shared code (firmware/common) against the Python reference, on the host.

- CRC and E2E receiver: bit-identical to kauha/can/e2e.py.
- Gateway decisions: identical to kauha/can/gateway.py on random frame streams with
  corrupted, repeated, late and missing frames.
- Wheel speeds: consistent with the twin's articulated kinematics.
- Generated files are current; the Teensy sketches compile against stub headers.
"""
from __future__ import annotations

import math
import random
import shutil
import subprocess
import sys

import pytest

from kauha.can.codec import Codec
from kauha.can.e2e import E2EReceiver, crc8_j1850, protect
from kauha.can.gateway import MachineGateway
from kauha.config import REPO_ROOT

FW = REPO_ROOT / "firmware"
COMMON = FW / "common"
pytestmark = pytest.mark.skipif(shutil.which("gcc") is None, reason="gcc not installed")


@pytest.fixture(scope="module")
def exe(tmp_path_factory):
    out = tmp_path_factory.mktemp("fw") / "host"
    srcs = [FW / "test" / "host_harness.c", COMMON / "kauha_e2e.c", COMMON / "gateway_core.c",
            COMMON / "drive_ctrl.c", COMMON / "generated" / "kauha_machine.c"]
    subprocess.run(["gcc", "-std=c99", "-O1", "-Wall", "-Werror", "-ffp-contract=off", f"-I{COMMON}",
                    *map(str, srcs), "-o", str(out), "-lm"], check=True, capture_output=True, text=True)
    return out


def run(exe, mode, lines):
    r = subprocess.run([str(exe), mode], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True)
    return r.stdout.splitlines()


def test_crc_matches_python(exe):
    rnd = random.Random(1)
    cases = [(rnd.randrange(256), [rnd.randrange(256) for _ in range(8)]) for _ in range(3000)]
    out = run(exe, "crc", [f"{i} " + " ".join(map(str, d)) for i, d in cases])
    assert [int(x) for x in out] == [crc8_j1850(bytes([i]) + bytes(d[:7])) for i, d in cases]


def test_e2e_receiver_matches_python(exe):
    rnd = random.Random(2)
    rx = E2EReceiver(0x10, 0.15)
    lines, want = ["16 " + (0.15).hex()], []
    t, ctr = 0.0, 0
    code = {"ok": 0, "ok_some_lost": 1, "repeated": 2, "wrong_sequence": 3, "crc_error": 4}
    for _ in range(3000):
        t += rnd.choice((0.01, 0.025, 0.05, 0.2))
        ctr = (ctr + rnd.choice((0, 1, 1, 1, 2, 3))) % 16
        f = bytearray(protect(bytearray(rnd.randrange(256) for _ in range(8)), 0x10, ctr))
        if rnd.random() < 0.05:
            f[rnd.randrange(8)] ^= 1 << rnd.randrange(8)
        new = rnd.random() < 0.8
        st = code[rx.check(bytes(f), t).value] if new else -1
        want.append(f"{st} {int(rx.fresh(t))}")
        lines.append(f"{t.hex()} {int(new)} " + " ".join(map(str, f)))
    assert run(exe, "e2e", lines) == want


def _frames(codec, rnd, t):
    """Random traffic: each message present with some probability, sometimes corrupted or replayed."""
    out = []
    if rnd.random() < 0.8:
        out.append(codec.encode("DriveCmd", dict(SpeedReq=round(rnd.uniform(-3.5, 3.5), 3),
                                                 ArticulationReq=round(rnd.uniform(-0.7, 0.7), 4),
                                                 Mode=rnd.choice((0, 1, 1, 2)), BrakeReq=int(rnd.random() < 0.2))))
    if rnd.random() < 0.8:
        out.append(codec.encode("ImplementCmd", dict(BoomRate=round(rnd.uniform(-1, 1), 2),
                                                     BucketRate=round(rnd.uniform(-1, 1), 2),
                                                     Hold=int(rnd.random() < 0.2))))
    if rnd.random() < 0.85:
        out.append(codec.encode("SafetyStatus", dict(State=rnd.choice((0, 0, 0, 1, 2, 3)),
                                                     SpeedLimit=round(rnd.uniform(0, 3.2), 3))))
    if rnd.random() < 0.1:
        out.append(codec.encode("MachineState1", dict(Speed=1.0)))           # not for the gateway
    res = []
    for fid, data in out:
        data = bytearray(data)
        if rnd.random() < 0.04:
            data[rnd.randrange(8)] ^= 1 << rnd.randrange(8)
        res.append((fid, bytes(data)))
        if rnd.random() < 0.03:
            res.append((fid, bytes(data)))                                   # repeated frame
    return res


def test_gateway_matches_python(exe):
    rnd = random.Random(3)
    codec = Codec()
    py = MachineGateway(codec)
    t = 0.0
    lines, want = [], []
    for _ in range(6000):
        t += rnd.choice((0.025, 0.025, 0.05, 0.12, 0.2))
        fr = _frames(codec, rnd, t)
        for fid, d in fr:
            py.receive(t, fid, d)
        c = py.command(t)
        want.append((c.speed, c.articulation, c.boom, c.bucket, c.brake, py.stats.clamped))
        lines.append(f"{t.hex()} {len(fr)} " + " ".join(f"{fid} " + " ".join(map(str, d)) for fid, d in fr))
    got = run(exe, "gw", lines)
    assert len(got) == len(want)
    brakes = 0
    for k, (g, w) in enumerate(zip(got, want, strict=True)):
        p = g.split()
        vals = (float.fromhex(p[0]), float.fromhex(p[1]), float.fromhex(p[2]), float.fromhex(p[3]), p[4] == "1",
                int(p[5]))
        assert vals[4] == w[4] and vals[5] == w[5], (k, vals, w)
        for a, b in zip(vals[:4], w[:4], strict=True):
            assert a == pytest.approx(b, abs=1e-9), (k, vals, w)
        brakes += w[4]
    assert 0 < brakes < len(want)          # both outcomes exercised


def test_wheel_speeds_follow_the_articulated_kinematics(exe):
    lf = lr = 0.2
    track = 0.2625
    rnd = random.Random(4)
    cases = [(rnd.uniform(-1.1, 1.1), rnd.uniform(-0.7, 0.7), rnd.uniform(-0.5, 0.5)) for _ in range(500)]
    out = run(exe, "wheels", [f"{lf.hex()} {lr.hex()} {track.hex()}"] + [" ".join(x.hex() for x in c) for c in cases])
    for (v, g, gd), line in zip(cases, out, strict=True):
        fl, fr, rl, rr, est = (float.fromhex(x) for x in line.split())
        yaw_f = (v * math.sin(g) + lr * gd) / (lf * math.cos(g) + lr)       # kauha/sim/loader.py
        assert (fr - fl) / track == pytest.approx(yaw_f, abs=1e-12)
        assert (fl + fr) / 2 == pytest.approx(v, abs=1e-12)
        assert (rr - rl) / track == pytest.approx(yaw_f - gd, abs=1e-12)
        if gd == 0:
            assert est == pytest.approx(v, abs=1e-12)


def test_generated_files_are_current(tmp_path):
    before = {p.name: p.read_text() for p in (COMMON / "generated").iterdir()}
    subprocess.run([sys.executable, str(FW / "generate.py")], check=True, capture_output=True)
    after = {p.name: p.read_text() for p in (COMMON / "generated").iterdir()}
    assert before == after, "run python firmware/generate.py and commit the result"


@pytest.mark.skipif(shutil.which("g++") is None, reason="g++ not installed")
@pytest.mark.parametrize("sketch", ["gateway", "sensor_node"])
def test_sketches_compile_against_stubs(sketch, tmp_path):
    r = subprocess.run(["g++", "-std=c++17", "-fsyntax-only", "-Wall", f"-I{FW / 'test' / 'stubs'}", f"-I{COMMON}",
                        "-x", "c++", str(FW / sketch / "src" / "main.cpp")], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-3000:]
