"""The four SIL processes.

machine  : loader dynamics, ray-cast safety scanners, a person, the drive-by-wire
           gateway, and the relays the PLC drives. If the PLC's relay signals
           stop arriving, the relays drop (de-energised = brake), as wired.
plc      : the safety PLC logic; reads the hardwired I/O link and the CAN bus.
autonomy : a minimal driver that requests a straight run at a set speed.
remote   : the supervision heartbeat.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import can
import numpy as np

from kauha.can.codec import Codec
from kauha.can.gateway import MachineGateway
from kauha.config import load_machine
from kauha.safety.fields import build_field_sets
from kauha.safety.plc import Mode, SafetyInputs, SafetyPLC
from kauha.safety.stopping import StopBudget
from kauha.sil.common import CYCLE_S, IOLink, Pacer, Wiring, drain, wait_for
from kauha.sim.loader import Loader, LoaderCommand, LoaderState
from kauha.sim.sensors import SafetyScanners

RELAY_TIMEOUT_S = 0.1          # relays drop if the PLC stops refreshing them
STARTUP_S = 1.5                # autonomy requests AUTO after this; supervisor resets just before
WALLS = np.array([(-200, -60, 400, -60), (-200, 60, 400, 60)], dtype=float)


def _send(bus: can.BusABC, codec: Codec, name: str, sig: dict) -> None:
    fid, data = codec.encode(name, sig)
    bus.send(can.Message(arbitration_id=fid, data=data, is_extended_id=True))


def machine(w: Wiring, epoch: float, duration: float, log_path: str, person: dict | None) -> None:
    """person: {'t': appear time, 'ahead': metres ahead of the bucket at appearance}."""
    bus, codec, gw = w.bus(), Codec(), MachineGateway()
    io = IOLink(w.io_to_machine, w.io_to_plc)
    cfg = load_machine()
    loader = Loader(cfg, LoaderState(), dt=CYCLE_S)
    loader.state.boom, loader.state.bucket = -0.2, 0.55
    scanners = SafetyScanners(np.random.default_rng(0))
    sets = build_field_sets(StopBudget())   # the field sets loaded into the scanners
    wait_for(epoch)
    pace = Pacer(CYCLE_S)
    t0 = epoch   # shared CLOCK_MONOTONIC epoch: all processes log on one time base
    px = None
    log = open(log_path, "w")
    scan = None
    while time.monotonic() - epoch < duration:
        t = time.monotonic() - t0
        s = loader.state
        for m in drain(bus):
            gw.receive(t, m.arbitration_id, bytes(m.data))
        relays = io.poll()
        relay_ok = relays is not None and io.age() < RELAY_TIMEOUT_S
        brake_release = relay_ok and relays.get("brake_release", False)
        implement_enable = relay_ok and relays.get("implement_enable", False)
        field_set = relays.get("field_set", 0) if relay_ok else 0
        if person and px is None and t >= person["t"]:
            px = s.x + loader.bucket_tip_local()[0] + person["ahead"]
        circles = np.array([[px, 0.0, 0.25]]) if px is not None else np.zeros((0, 3))
        if pace.k % 2 == 0 or scan is None:
            rx, ry, rth = loader.rear_axle()
            scan = scanners.scan({"front": (s.x, s.y, s.theta), "rear": (rx, ry, rth + math.pi)},
                                 sets[field_set], WALLS, circles)
        io.send({"front_prot": scan.front_prot, "front_warn": scan.front_warn, "rear_prot": scan.rear_prot,
                 "rear_warn": scan.rear_warn, "side_prot": scan.side_prot, "side_contact": scan.side_contact,
                 "speed_b": s.v,
                 "articulation": s.gamma, "t": t})
        cmd = gw.command(t)
        if not brake_release:
            cmd = LoaderCommand(speed=0.0, articulation=s.gamma, brake=True)
        if not implement_enable:
            cmd.boom = cmd.bucket = 0.0
        loader.step(cmd)
        _send(bus, codec, "MachineState1", {"Speed": float(np.clip(s.v, -5.9, 5.9)),
                                            "Articulation": float(np.clip(s.gamma, -0.79, 0.79)),
                                            "Boom": float(np.clip(s.boom, -0.99, 0.99))})
        gap = None
        if px is not None:
            gap = px - 0.25 - (s.x + loader.bucket_tip_local()[0])
        log.write(json.dumps({"t": round(t, 3), "x": round(s.x, 3), "v": round(s.v, 3), "relay_ok": relay_ok,
                              "brake_release": brake_release, "gw_brake": cmd.brake, "gap": gap,
                              "gw_timeouts": gw.stats.timeouts}) + "\n")
        pace.wait()
    log.write(json.dumps({"overruns": pace.overruns}) + "\n")
    log.close()
    bus.shutdown()


def plc(w: Wiring, epoch: float, duration: float, log_path: str) -> None:
    bus, codec = w.bus(), Codec()
    io = IOLink(w.io_to_plc, w.io_to_machine)
    rx_drive, rx_hb = codec.receiver("DriveCmd", 0.15), codec.receiver("RemoteHeartbeat", 0.5)
    plc_ = SafetyPLC()
    drive: dict = {}
    hb: dict = {}
    wait_for(epoch)
    pace = Pacer(CYCLE_S)
    t0 = epoch   # shared CLOCK_MONOTONIC epoch: all processes log on one time base
    while time.monotonic() - epoch < duration:
        t = time.monotonic() - t0
        for m in drain(bus):
            try:
                name, sig = codec.decode(m.arbitration_id, bytes(m.data))
            except KeyError:
                continue
            if name == "DriveCmd" and rx_drive.check(bytes(m.data), t).value.startswith("ok"):
                drive = sig
            elif name == "RemoteHeartbeat" and rx_hb.check(bytes(m.data), t).value.startswith("ok"):
                hb = sig
        sensors = io.poll() or {}
        hb_age = 99.0 if rx_hb.last_ok_time is None else t - rx_hb.last_ok_time
        cmd_age = 99.0 if rx_drive.last_ok_time is None else t - rx_drive.last_ok_time
        # Hardwired inputs lost (cable cut) count as violated fields: fail-safe.
        io_ok = io.age() < 0.1
        v = sensors.get("speed_b", 0.0)
        out = plc_.step(SafetyInputs(
            t=t, mode=Mode(int(drive.get("Mode", 0))), speed_a=v, speed_b=v,
            articulation=sensors.get("articulation", 0.0),
            front_prot=sensors.get("front_prot", True) or not io_ok, front_warn=sensors.get("front_warn", False),
            rear_prot=sensors.get("rear_prot", False), rear_warn=sensors.get("rear_warn", False),
            side_prot=sensors.get("side_prot", False), side_contact=sensors.get("side_contact", True) or not io_ok,
            remote_hb_age=hb_age,
            remote_estop=bool(hb.get("EStop", 0)) and hb_age < 0.5, reset_req=bool(hb.get("ResetReq", 0)),
            autonomy_cmd_age=cmd_age))
        io.send({"brake_release": out.brake_release, "implement_enable": out.implement_enable,
                 "field_set": out.field_set})
        _send(bus, codec, "SafetyStatus", {"State": int(out.state), "ActiveFieldSet": min(out.field_set, 255),
                                           "Cause": int(out.cause) & 0xFFFF,
                                           "SpeedLimit": max(0.0, min(6.0, out.speed_limit)),
                                           "ResetPending": int(out.reset_pending)})
        pace.wait()
    Path(log_path).write_text(json.dumps(plc_.log.records))
    bus.shutdown()


def autonomy(w: Wiring, epoch: float, duration: float, speed: float) -> None:
    bus, codec = w.bus(), Codec()
    wait_for(epoch)
    pace = Pacer(0.05)
    while time.monotonic() - epoch < duration:
        drain(bus)
        # Start in IDLE; request AUTO once the supervisor has had time to commission.
        mode = 1 if time.monotonic() - epoch > STARTUP_S else 0
        _send(bus, codec, "DriveCmd", {"SpeedReq": speed, "ArticulationReq": 0.0, "Mode": mode,
                                       "BrakeReq": int(mode == 0)})
        _send(bus, codec, "ImplementCmd", {"BoomRate": 0.0, "BucketRate": 0.0, "Hold": 0})
        pace.wait()
    bus.shutdown()


def remote(w: Wiring, epoch: float, duration: float) -> None:
    bus, codec = w.bus(), Codec()
    wait_for(epoch)
    pace = Pacer(0.05)
    while time.monotonic() - epoch < duration:
        drain(bus)
        # Start-of-shift: the supervisor checks the cameras and presses reset once.
        t = time.monotonic() - epoch
        _send(bus, codec, "RemoteHeartbeat", {"EStop": 0, "ResetReq": int(STARTUP_S - 0.6 < t < STARTUP_S - 0.3),
                                              "ModeReq": 1})
        pace.wait()
    bus.shutdown()
