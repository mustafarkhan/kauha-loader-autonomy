"""Evaluate the validation criteria (kauha/validation/criteria.py).

Each check returns a Result with a status:
  pass        the target is met
  fail        the target is not met (kept in the table; see the note)
  finding     the check exposed a design limit that is not a numeric miss
  open        cannot be decided from what is available (says what is missing)
  needs data  the dataset directory is not set
"""
from __future__ import annotations

import json
import math
import os
import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from kauha.can.codec import Codec
from kauha.can.e2e import E2EReceiver, E2EStatus, protect
from kauha.can.gateway import MachineGateway
from kauha.config import REPO_ROOT, load_machine
from kauha.safety.fields import build_field_sets, point_in_polygon
from kauha.safety.stopping import StopBudget

G = 9.81


@dataclass
class Result:
    status: str
    measured: str
    note: str = ""


def _data(env: str) -> Path | None:
    p = os.environ.get(env)
    return Path(p) if p and Path(p).is_dir() else None


# ------------------------------------------------------------------ standards
def v_std_01() -> Result:
    need = G * math.sin(math.atan(0.25))
    dry, winter = StopBudget().decel_m_s2, StopBudget().winter().decel_m_s2
    ok = dry >= need
    return Result("pass" if ok else "fail", f"assumed dry braking {dry:.2f} m/s2 against {need:.2f} needed",
                  f"margin {100 * (dry / need - 1):.0f} %. The winter planning value ({winter:.2f} m/s2) is the "
                  "available friction on a slippery surface, not the brake's capacity, so it is not compared here. "
                  "The brake itself has to be measured on the machine.")


def v_std_02() -> Result:
    m, b = load_machine(), StopBudget()
    v = m.max_speed
    kmh = v * 3.6
    brake = v * b.brake_buildup_s + v * v / (2 * b.decel_m_s2)   # from brake actuation, as the standard measures
    chain = brake + v * b.actuator_latency_s                        # with the retrofit's brake actuator delay
    candidate = kmh ** 2 / 150 + 0.2 * (kmh + 5)
    need = v * v / (2 * (candidate - v * b.brake_buildup_s))
    return Result("open", f"twin stops from {kmh:.1f} km/h in {brake:.1f} m after brake actuation "
                  f"({chain:.1f} m with the retrofit actuator's delay)",
                  "Confirmed from the 1985 edition's sample: the test speed is 32 km/h or the top speed if lower, and "
                  "the distance is measured from the point where the brake control is actuated. The limit table is "
                  "not legible there and later editions are not free. The commonly quoted later form "
                  f"(v2/150 + 0.2(v + 5), v in km/h) gives {candidate:.1f} m, which the assumed braking would "
                  f"{'meet' if brake <= candidate else 'miss'} measured the standard's way; it corresponds to a mean "
                  f"deceleration of about {need:.1f} m/s2, close to the {b.decel_m_s2:.1f} m/s2 the stop budget "
                  "assumes. To close: read the limit in the purchased standard and measure the machine's brake.")


def v_std_03() -> Result:
    from kauha.adversary.search import decode
    fastest = decode(np.array([0.5] * 8 + [1.0, 0.0])).speed
    return Result("pass" if fastest >= 1.6 else "fail", f"adversarial walker speeds up to {fastest:.1f} m/s",
                  "The fields are sized for the machine's stopping distance, as ISO 3691-4 does; a person walking "
                  "into a machine that has already stopped is outside the stop function.")


def _far_points(poly: list) -> list[tuple[float, float]]:
    """Points 0.15 m inside the far end of a field: on its axis and near both far corners."""
    xs = [p[0] for p in poly]
    far = [p for p in poly if p[0] > max(xs) - 0.35 * (max(xs) - min(xs))]
    cx, cy = sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly)
    cands = [max(far, key=lambda p: p[0]), max(far, key=lambda p: p[1]), min(far, key=lambda p: p[1])]
    out = []
    for x, y in cands:
        d = math.hypot(cx - x, cy - y)
        for step in (0.25, 0.4, 0.6):
            q = (x + (cx - x) * step / d, y + (cy - y) * step / d)
            if point_in_polygon(q[0], q[1], poly):
                out.append(q)
                break
    return out


def v_std_04() -> Result:
    from kauha.sim.sensors import SafetyScanners
    m = load_machine()
    sets = build_field_sets(StopBudget())
    poses = {"front": (0.0, 0.0, 0.0), "rear": (-(m.lf + m.lr), 0.0, math.pi)}
    total, missed = 0, []
    for fs in sets.values():
        for body, poly in (("front", fs.protective if fs.direction >= 0 else []),
                           ("rear", fs.protective_for("rear") if fs.direction <= 0 else [])):
            if not poly:
                continue
            for bx, by in _far_points(poly):
                px, py, pth = poses[body]
                c, s = math.cos(pth), math.sin(pth)
                circle = np.array([[px + c * bx - s * by, py + s * bx + c * by, 0.10]])
                sc = SafetyScanners(np.random.default_rng(0))
                for _ in range(2):                   # a field counts after two consecutive scans
                    r = sc.scan(poses, fs, np.zeros((0, 4)), circle)
                total += 1
                if not (r.front_prot if body == "front" else r.rear_prot):
                    missed.append(f"{fs.name}/{body} at ({bx:.2f}, {by:.2f})")
    # every stopping field must lie inside the rated coverage of a safety sensor
    from kauha.safety.coverage import reach, scanner_only_speed_cap, uncovered
    gaps = uncovered(sets)
    far = reach(sets)
    note = (f"2D twin; a return counts only inside a sensor's rated coverage. The front and rear fields end up to "
            f"{far['scanner']:.1f} m from the scanners, beyond their 4 m rated range, so three 9 m safety radars per "
            f"end watch the far part ({far['radar']:.1f} m at most); the scanners keep the side, contact and "
            f"under-bucket fields. With the scanners alone the forward cap would be "
            f"{scanner_only_speed_cap(StopBudget()):.2f} m/s. The radar is rated for people, not for the test "
            "piece, and its use on a moving outdoor machine is to be confirmed with the manufacturer; the check "
            "on the hardware is test T4.")
    if missed or gaps:
        where = "; ".join(missed[:6] + [f"{a}/{b} uncovered at ({x:.2f}, {y:.2f})" for a, b, x, y in gaps[:6]])
        return Result("fail", f"{total - len(missed)} of {total} test-piece positions detected; "
                      f"{len(gaps)} field sample points outside rated coverage", where + ". " + note)
    return Result("pass", f"{total} of {total} test-piece positions detected in {len(sets)} field sets; "
                  "0 field sample points outside rated coverage", note)


def v_std_05() -> Result:
    from kauha.safety.fields import SCAN_PLANE_M
    return Result("finding", f"scan plane {SCAN_PLANE_M} m on the full machine; no rated means of detecting a person "
                  "lying on the ground",
                  "Decided: accepted as a stated residual risk (safety concept, residual risk 11). A scan plane at "
                  "70 mm cannot be used on a loader: on gravel and ruts it would see the ground. ISO 3691-4 is "
                  "written for driverless trucks on flat floors; for autonomous earth-moving machines ISO 17757 "
                  "relies on a controlled autonomous operating zone. Measures: access control to the operating "
                  "area (A1). The autonomy side's 3D lidar and camera are the only sensors that could see a person "
                  "on the ground; they are not safety rated and that case is not tested. The 1:8 testbed's "
                  "plane is about 0.085 m, so test T4.2 shows the limit rather than hiding it.")


def v_std_06() -> Result:
    rep = json.loads((REPO_ROOT / "hardware" / "scale" / "cad" / "export" / "report.json").read_text())
    st = rep["stability"]
    sand = rep["bucket_struck_volume_l"] * 1.6
    ok = st["rated_operating_load_kg"] >= sand
    return Result("pass" if ok else "fail",
                  f"rated load {st['rated_operating_load_kg']} kg, tipping load {st['tipping_load_full_turn_kg']} kg, "
                  f"bucket of sand {sand:.1f} kg", "From CAD mass properties; weigh the built machine to confirm.")


def v_std_07() -> Result:
    from kauha.safety.pl import PL_D_LIMIT, stop_chain
    c = stop_chain()
    missing = [p.name for p in c["parts"] if p.pfhd is None]
    known = "; ".join(f"{p.name} {p.pfhd:.2e}" for p in c["parts"] if p.pfhd is not None)
    measured = (f"selected parts add up to {c['known_pfhd']:.2e} 1/h of the {PL_D_LIMIT:.0e} PL d limit; "
                f"{c['budget_for_unselected']:.2e} 1/h left for the parts not selected")
    note = (f"{known}. Output stage MTTFd {c['mttfd_output_years']:.0f} years per channel at "
            f"{c['operations_per_year']:.0f} operations a year; common-cause score claimed {c['ccf_score']} "
            f"(65 needed). Not selected, so no figure: {'; '.join(missing)}. PL d holds if those stay inside the "
            "remaining budget. Table K.1 values are from a manufacturer's reproduction and need confirming in the "
            "standard. kauha/safety/pl.py.")
    ok = not missing and c["known_pfhd"] < PL_D_LIMIT and c["ccf_score"] >= 65
    return Result("pass" if ok else "open", measured, note)


# ------------------------------------------------------------------ published measurements
def v_pub_01() -> Result:
    t = json.loads((REPO_ROOT / "models" / "bucket_training.json").read_text())["test"]["randomised"]["learned"]
    ok = t["time_s_mean"] <= 8.82 and t["fill_factor_mean"] >= 0.90
    return Result("pass" if ok else "fail",
                  f"fill time {t['time_s_mean']:.1f} s, fill factor {t['fill_factor_mean']:.2f} "
                  f"({t['episodes']} held-out episodes)",
                  "Simulated bucket-pile model against a field result on a larger machine (L180H); it shows the "
                  "policy is in the published range, not that the model is right.")


# the gravel test material and test seed of kauha/train/train_bucket.py (importing it needs torch)
GRAVEL = dict(material="gravel", density_t_m3=1.75, k_resist=1.35, repose_deg=36, height_m=3.0)
TEST_SEED = 20000


def v_pub_02(n: int = 40) -> Result:
    from kauha.autonomy.bucket_policy import BucketPolicy
    from kauha.train.bucket_env import evaluate
    act = BucketPolicy.load(REPO_ROOT / "models" / "bucket_policy.npz").act
    rows = []
    for scale in (0.89, 1.0, 1.11):
        mat = dict(GRAVEL)
        mat["k_resist"] *= scale
        r = evaluate(act, n=n, seed=TEST_SEED, env_kwargs={"fixed_material": mat})
        rows.append((scale, r["fill_factor_mean"], r["success_rate"], r["time_s_mean"]))
    ok = all(f >= 0.85 and s == 1.0 for _, f, s, _ in rows)
    return Result("pass" if ok else "fail",
                  "; ".join(f"resistance x{k:.2f}: fill {f:.2f}, {t:.1f} s, success {100 * s:.0f} %"
                            for k, f, s, t in rows),
                  "Covers the published sim-to-real force error as a change in digging resistance only.")


def v_pub_03() -> Result:
    camp = json.loads((REPO_ROOT / "results" / "campaign.json").read_text())
    ours = camp["learned"]["summary"]["cycle_time_median_s"]["mean"]
    return Result("finding", f"KAUHA median cycle {ours:.0f} s (truck loading, twin); every operator in the study's "
                  "plots is below about 45 s",
                  "The study gives cycle times only in figures 9 and 10, whose axis ends near 45 s, and reports "
                  "spread as ratios: productivity differs by a factor of 5 to 8 between operators, 2 to 4 without "
                  "novices. So the twin's autonomous cycle is at least 1.5 times as long as a human short loading "
                  "cycle. The causes are the 3 m/s speed cap from the protective field length and a longer travel "
                  "path in the twin's site. The honest claim is unattended running, not speed. A like-for-like "
                  "figure needs the pilot site's own operator timing.")


def v_pub_04() -> Result:
    d = _data("KAUHA_DATA_TUNI_LOADER")
    return Result("needs data" if d is None else "open", "replay not built",
                  "The recording's channel list is in its documentation PDF, which could not be read from the build "
                  "environment. The replay will be written against that list, not guessed.")


# ------------------------------------------------------------------ machine bus under attack
def _legit(codec: Codec, state: int, limit: float, speed: float = 2.0):
    return [codec.encode("DriveCmd", dict(SpeedReq=speed, ArticulationReq=0.0, Mode=1, BrakeReq=0)),
            codec.encode("ImplementCmd", dict(BoomRate=0.0, BucketRate=0.0, Hold=0)),
            codec.encode("SafetyStatus", dict(State=state, SpeedLimit=limit))]


def direct_fuzzing(n: int = 200_000, seed: int = 1) -> dict:
    """Attacker on the safety segment itself (inside the cabinet): random payloads on a protected
    identifier, between genuine frames, straight at a receiver."""
    rnd = random.Random(seed)
    rx = E2EReceiver(0x10, 0.15)
    ctr, accepted = 0, 0
    for k in range(n):
        if k % 10 == 0:                              # one genuine frame per ten forged
            ctr = (ctr + 1) % 16
            rx.check(protect(bytearray(8), 0x10, ctr), k * 0.005)
        st = rx.check(bytes(rnd.randrange(256) for _ in range(8)), k * 0.005)
        accepted += st in (E2EStatus.OK, E2EStatus.OK_SOME_LOST)
    return {"forged": n, "accepted": accepted, "rate": accepted / n}


def direct_masquerade(cycles: int = 400) -> dict:
    """An attacker who knows the (public) CRC scheme forges SafetyStatus RUN after every genuine STOP."""
    codec = Codec()
    gw = MachineGateway(codec)
    sid = codec.db.get_message_by_name("SafetyStatus")
    did = codec.data_id("SafetyStatus")
    unsafe, t = 0, 0.0
    for _ in range(cycles):
        t += 0.025
        frames = _legit(codec, state=2, limit=0.0)
        for fid, d in frames:
            gw.receive(t, fid, d)
        real_ctr = frames[2][1][6] & 0x0F
        sig = {s.name: 0 for s in sid.signals}
        sig.update(State=0, SpeedLimit=3.0)
        forged = protect(bytearray(sid.encode(sig, strict=True)), did, (real_ctr + 1) % 16)
        gw.receive(t + 0.001, frames[2][0], forged)
        cmd = gw.command(t + 0.002)
        unsafe += (not cmd.brake) and cmd.speed > 0
    return {"cycles": cycles, "gateway_released_while_plc_stopped": unsafe}


def direct_replay(n: int = 20_000, seed: int = 2) -> dict:
    """Genuine frames recorded earlier and sent again at random times."""
    rnd = random.Random(seed)
    rx = E2EReceiver(0x10, 0.15)
    recorded = [protect(bytearray(rnd.randrange(256) for _ in range(8)), 0x10, c % 16) for c in range(64)]
    ctr, accepted = 0, 0
    for k in range(n):
        ctr = (ctr + 1) % 16
        rx.check(protect(bytearray(8), 0x10, ctr), k * 0.05)
        st = rx.check(rnd.choice(recorded), k * 0.05 + 0.01)
        accepted += st in (E2EStatus.OK, E2EStatus.OK_SOME_LOST)
    return {"replayed": n, "accepted": accepted, "rate": accepted / n}


# The attacks below come from the open segment (autonomy computer, remote link, zone source), which
# is where a ROAD-style attacker sits: a device on a connector or a compromised networked unit.
# Every frame from there passes the segment guard (kauha/can/guard.py).
CHALLENGE = bytes.fromhex("0102030405060708")


def _open_side(challenge: bytes = CHALLENGE):
    from kauha.can.auth import TEST_KEYS, AuthSender
    from kauha.can.guard import SegmentGuard
    guard = SegmentGuard(dict(TEST_KEYS), challenge)
    senders = {sa: AuthSender(sa, k) for sa, k in TEST_KEYS.items()}
    for sd in senders.values():
        sd.on_challenge(challenge)
    return guard, senders


def _genuine(guard, sender, t: float, fid: int, data: bytes):
    """Send one genuine frame and its companion through the guard; returns what the guard forwards."""
    guard.from_open(t, fid, data)
    return guard.from_open(t + 0.0005, *sender.sign(fid, data))


def attack_fuzzing(n: int = 200_000, seed: int = 1) -> dict:
    """Random frames on the open segment, between genuine commands: random payloads on the command
    identifiers, on the companion identifiers, on safety-segment identifiers and on random identifiers."""
    from kauha.can.auth import auth_frame_id
    rnd = random.Random(seed)
    codec = Codec()
    guard, senders = _open_side()
    ids = [codec.db.get_message_by_name(m).frame_id for m in
           ("DriveCmd", "ImplementCmd", "RemoteHeartbeat", "ZoneStatus", "SafetyStatus", "SafeSensors",
            "ScannerStatus", "MachineState1")] + [auth_frame_id(sa) for sa in (0x80, 0x82, 0x86)]
    genuine, forged_through, sent, got = set(), 0, 0, 0
    for k in range(n):
        t = k * 0.0025
        fid = rnd.choice(ids) if rnd.random() < 0.9 else rnd.getrandbits(29)
        forged = (fid, bytes(rnd.randrange(256) for _ in range(8)))
        outs = []
        if k % 10 == 0:                              # one genuine command per ten forged frames
            gid, d = codec.encode("DriveCmd", dict(SpeedReq=1.0, Mode=1))
            genuine.add((gid, d))
            sent += 1
            guard.from_open(t, gid, d)
            if k % 20 == 0:                          # every second time the forged frame lands inside the pair
                outs.append(guard.from_open(t + 0.0002, *forged))
            ok = guard.from_open(t + 0.0005, *senders[0x80].sign(gid, d))
            got += ok is not None
            if k % 20 == 0:
                continue
        outs.append(guard.from_open(t + 0.001, *forged))
        forged_through += sum(o is not None and o not in genuine for o in outs)
    return {"forged": n, "accepted": forged_through, "rate": forged_through / n,
            "genuine_delivered": got / sent, "intrusions": guard.stats.intrusions}


def attack_masquerade(cycles: int = 400) -> dict:
    """An attacker who knows the CRC scheme but has no key: forges SafetyStatus RUN after every genuine
    STOP, and a RemoteHeartbeat (no e-stop) with a companion frame made with a guessed key."""
    from kauha.can.auth import AuthSender
    codec, gw = Codec(), MachineGateway(Codec())
    guard, senders = _open_side()
    wrong = AuthSender(0x82, bytes(16))
    wrong.on_challenge(CHALLENGE)
    sid = codec.db.get_message_by_name("SafetyStatus")
    hb = codec.db.get_message_by_name("RemoteHeartbeat")
    unsafe, forged_hb, t = 0, 0, 0.0
    for cyc in range(cycles):
        t += 0.025
        drive, impl, status = _legit(codec, state=2, limit=0.0)
        for fid, d in (drive, impl):                 # commands come from the open segment
            out = _genuine(guard, senders[0x80], t, fid, d)
            if out:
                gw.receive(t, *out)
        gw.receive(t, *status)                       # the PLC is on the safety segment
        sig = {s.name: 0 for s in sid.signals}
        sig.update(State=0, SpeedLimit=3.0)
        forged = protect(bytearray(sid.encode(sig, strict=True)), codec.data_id("SafetyStatus"),
                         ((status[1][6] & 0x0F) + 1) % 16)
        out = guard.from_open(t + 0.001, status[0], forged)
        if out:
            gw.receive(t + 0.001, *out)
        fake = protect(bytearray(hb.encode({s.name: 0 for s in hb.signals}, strict=True)),
                       codec.data_id("RemoteHeartbeat"), cyc % 16)
        guard.from_open(t + 0.002, hb.frame_id, fake)
        forged_hb += guard.from_open(t + 0.0025, *wrong.sign(hb.frame_id, fake)) is not None
        cmd = gw.command(t + 0.003)
        unsafe += (not cmd.brake) and cmd.speed > 0
    return {"cycles": cycles, "gateway_released_while_plc_stopped": unsafe, "forged_heartbeats_accepted": forged_hb,
            "intrusions": guard.stats.intrusions}


def attack_replay(n: int = 20_000, seed: int = 2) -> dict:
    """Genuine command and companion frames recorded earlier and sent again: in the same session, and
    after the guard has restarted with a new challenge."""
    rnd = random.Random(seed)
    codec = Codec()
    guard, senders = _open_side()
    recorded, accepted = [], 0
    for k in range(64):
        fid, d = codec.encode("DriveCmd", dict(SpeedReq=2.0, Mode=1))
        recorded.append(((fid, d), senders[0x80].sign(fid, d)))
        guard.from_open(k * 0.025, fid, d)
        guard.from_open(k * 0.025 + 0.0005, *recorded[-1][1])
    for k in range(n // 2):
        t = 2.0 + k * 0.05
        frame, auth = rnd.choice(recorded)
        guard.from_open(t, *frame)
        accepted += guard.from_open(t + 0.0005, *auth) is not None
    guard2, _ = _open_side(bytes.fromhex("a1a2a3a4a5a6a7a8"))       # guard power cycle: new challenge
    for k in range(n // 2):
        t = k * 0.05
        frame, auth = rnd.choice(recorded)
        guard2.from_open(t, *frame)
        accepted += guard2.from_open(t + 0.0005, *auth) is not None
    return {"replayed": n, "accepted": accepted, "rate": accepted / n}


def attack_suspension() -> dict:
    """All frames stop (bus-off, cut wire): time until the gateway brakes."""
    codec = Codec()
    gw = MachineGateway(codec)
    t = 0.0
    for _ in range(40):
        t += 0.025
        for fid, d in _legit(codec, state=0, limit=3.0):
            gw.receive(t, fid, d)
    assert not gw.command(t).brake
    t_stop = t
    while not gw.command(t_stop).brake:
        t_stop += 0.005
    return {"brake_after_s": round(t_stop - t, 3)}


def v_dat_01() -> Result:
    fz, mq, rp, su = attack_fuzzing(), attack_masquerade(), attack_replay(), attack_suspension()
    inside = direct_fuzzing()
    measured = (f"from the open segment, through the segment guard: fuzzing {fz['accepted']} of {fz['forged']} "
                f"forged frames forwarded; replay {rp['accepted']} of {rp['replayed']} accepted; masquerade: gateway "
                f"released in {mq['gateway_released_while_plc_stopped']} of {mq['cycles']} cycles while the PLC said "
                f"stop, {mq['forged_heartbeats_accepted']} forged heartbeats accepted; suspension: brake after "
                f"{su['brake_after_s']} s")
    ok = (fz["accepted"] == 0 and rp["accepted"] == 0 and mq["gateway_released_while_plc_stopped"] == 0
          and mq["forged_heartbeats_accepted"] == 0 and su["brake_after_s"] <= 0.15)
    return Result("pass" if ok else "fail", measured,
                  "The bus is split: the PLC, sensors and gateway are on a safety segment inside the cabinet; the "
                  "autonomy computer, remote link and zone source are on an open segment. The guard forwards only "
                  "four command frames from the open segment, each authenticated with an AES-128 CMAC, a freshness "
                  "counter and a per-power-cycle session key. Under fuzzing "
                  f"{100 * fz['genuine_delivered']:.0f} % of genuine commands still got through; the rest are lost "
                  "and end in a stop by timeout. Limits: a compromised autonomy computer holds its own key and can "
                  "send any command, which the PLC bounds as before. An attacker with a tool inside the cabinet is "
                  f"on the safety segment, where the CRC alone does not stop forged frames ({inside['accepted']} of "
                  f"{inside['forged']} random frames accepted at a receiver); that is covered by physical access "
                  "control, not by the bus. Reference implementation and firmware port, tested against each other; "
                  "not yet run on the hardware.")


def parse_candump(path: Path, limit: int = 200_000):
    """Lines of `(time) iface ID#DATA` (candump -L), as the ROAD logs use."""
    out = []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 3 or "#" not in parts[2]:
                continue
            ident, data = parts[2].split("#", 1)
            if len(data) != 16:
                continue
            out.append((float(parts[0].strip("()")), int(ident, 16), bytes.fromhex(data)))
            if len(out) >= limit:
                break
    return out


def replay_log_on_bus(frames) -> dict:
    """Real traffic onto the KAUHA bus: as recorded, then with identifiers remapped onto KAUHA's."""
    codec = Codec()
    gw = MachineGateway(codec)
    for t, fid, d in frames:
        gw.receive(t, fid, d)
    as_recorded = bool(frames) and not gw.command(frames[-1][0]).brake      # must stay False
    ids = [codec.db.get_message_by_name(n).frame_id for n in ("DriveCmd", "ImplementCmd", "SafetyStatus")]
    rxs = {i: E2EReceiver((i >> 8) & 0xFF, 0.15) for i in ids}
    accepted = 0
    for k, (t, _fid, d) in enumerate(frames):
        i = ids[k % 3]
        accepted += rxs[i].check(d, t) in (E2EStatus.OK, E2EStatus.OK_SOME_LOST)
    # the same remapped traffic from the open segment, as command and companion identifiers
    from kauha.can.auth import auth_frame_id
    guard, _ = _open_side()
    open_ids = ids[:2] + [auth_frame_id(0x80)]
    forwarded = sum(guard.from_open(t, open_ids[k % 3], d) is not None for k, (t, _fid, d) in enumerate(frames))
    return {"frames": len(frames), "decoded_as_recorded": as_recorded, "accepted_remapped": accepted,
            "forwarded_by_guard": forwarded}


def v_dat_02() -> Result:
    d = _data("KAUHA_DATA_ROAD")
    if d is None:
        return Result("needs data", "", "Set KAUHA_DATA_ROAD to a folder of candump logs.")
    logs = sorted(d.rglob("*.log"))[:20]
    tot = {"frames": 0, "accepted_remapped": 0, "forwarded_by_guard": 0}
    for p in logs:
        r = replay_log_on_bus(parse_candump(p))
        for k in tot:
            tot[k] += r[k]
    ok = tot["forwarded_by_guard"] == 0
    return Result("pass" if ok else "fail", f"{tot['forwarded_by_guard']} of {tot['frames']} frames forwarded by the "
                  f"segment guard; {tot['accepted_remapped']} would pass the CRC at a receiver ({len(logs)} logs)",
                  "Recorded vehicle traffic remapped onto KAUHA identifiers. It carries no KAUHA CRC or key, so "
                  "passing the CRC is by chance and passing the guard needs a valid 32-bit authentication code.")


# ------------------------------------------------------------------ perception datasets
def v_dat_03() -> Result:
    d = _data("KAUHA_DATA_SODA")
    if d is None:
        return Result("needs data", "", "Set KAUHA_DATA_SODA to a YOLO-layout copy (images/, labels/).")
    try:
        from ultralytics import YOLO

        from kauha.train.train_ppe import person_rates
    except ImportError:
        return Result("open", "", "Install the train extra (ultralytics) to run the detector.")
    cls = int(os.environ.get("KAUHA_SODA_PERSON_CLASS", "0"))
    r = person_rates(YOLO(str(REPO_ROOT / "models" / "ppe_yolo11n.pt")), d / "images", d / "labels", cls, 416, 0.25)
    ok = r["recall"] >= 0.80 and r["precision"] >= 0.75
    return Result("pass" if ok else "fail", f"recall {r['recall']:.2f}, precision {r['precision']:.2f}",
                  "Different site, camera and labels from the training set.")


def dust_frame_false_occupied(points: np.ndarray, boxes: list, field: list, z_band=(-0.15, 0.15),
                              min_points: int = 2) -> tuple[bool, bool]:
    """One lidar frame against one protective field placed at the sensor (x forward).

    Returns (occupied, object_present): occupied = at least `min_points` returns of the
    scan-height slice inside the field; object_present = a labelled box centre inside it.
    """
    sl = points[(points[:, 2] >= z_band[0]) & (points[:, 2] <= z_band[1])]
    inside = sum(point_in_polygon(float(x), float(y), field) for x, y in sl[:, :2])
    present = any(point_in_polygon(b[0], b[1], field) for b in boxes)
    return inside >= min_points, present


def read_dustx_frame(bin_path: Path):
    """LiDARDustX frame: .bin float32 x, y, z, intensity; .txt `class h w l x y z angle` per box."""
    pts = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    boxes = []
    txt = bin_path.with_suffix(".txt")
    if txt.exists():
        for line in txt.read_text().splitlines():
            p = line.split()
            if len(p) >= 8:
                boxes.append((float(p[4]), float(p[5]), p[0]))
    return pts, boxes


def v_dat_04(max_frames: int = 2000) -> Result:
    d = _data("KAUHA_DATA_LIDARDUSTX")
    if d is None:
        return Result("needs data", "", "Set KAUHA_DATA_LIDARDUSTX to the dataset folder (research-use licence).")
    field = build_field_sets(StopBudget())[8].protective        # forward, 1.5 m/s band, straight
    x0 = min(p[0] for p in field)
    field = [(x - x0 + 0.5, y) for x, y in field]                # field starts 0.5 m ahead of the sensor
    rows = []
    for sensor in sorted(p for p in d.iterdir() if p.is_dir()):
        n = false = 0
        for b in sorted(sensor.rglob("*.bin"))[:max_frames]:
            occ, present = dust_frame_false_occupied(*read_dustx_frame(b), field)
            n += 1
            false += occ and not present
        if n:
            rows.append((sensor.name, n, false / n))
    worst = max((r[2] for r in rows), default=0.0)
    return Result("pass" if worst < 0.01 else "fail",
                  "; ".join(f"{s}: {100 * r:.1f} % of {n} frames" for s, n, r in rows),
                  "No dust filter applied; the sensors are automotive lidars, not safety scanners, and their "
                  "mounting height is assumed to put the slice near KAUHA's scan plane.")


# ------------------------------------------------------------------ snow (Boreas) and radar (RADIATE)
def read_boreas_frame(bin_path: Path) -> np.ndarray:
    """Boreas Velodyne frame: float32 x, y, z, intensity, laser number, time (pyboreas load_lidar)."""
    return np.fromfile(bin_path, dtype=np.float32).reshape(-1, 6)


def boreas_occupied_rate(seq: Path, field: list, sensor_height: float, max_frames: int) -> tuple[int, float]:
    """Share of frames with returns inside the field at KAUHA's scan height (0.5 m above the road)."""
    z = SCAN_Z - sensor_height
    frames = sorted((seq / "lidar").glob("*.bin"))[:max_frames] if (seq / "lidar").is_dir() else []
    occ = sum(dust_frame_false_occupied(read_boreas_frame(f)[:, :4], [], field, z_band=(z - 0.15, z + 0.15))[0]
              for f in frames)
    return len(frames), (occ / len(frames) if frames else 0.0)


SCAN_Z = 0.5


def v_dat_05(max_frames: int = 1500) -> Result:
    d = _data("KAUHA_DATA_BOREAS")
    if d is None or not (d / "snow").is_dir() or not (d / "clear").is_dir():
        return Result("needs data", "", "Set KAUHA_DATA_BOREAS to a folder with two subfolders, snow/ and clear/, "
                      "each holding Boreas sequences of the same route (boreas-YYYY-MM-DD-HH-MM/lidar/*.bin). "
                      "Optional KAUHA_BOREAS_SENSOR_HEIGHT in metres (default 2.0).")
    h = float(os.environ.get("KAUHA_BOREAS_SENSOR_HEIGHT", "2.0"))
    field = [(2.5, -1.9), (6.5, -1.9), (6.5, 1.9), (2.5, 1.9)]     # 4 m field starting ahead of the car's nose
    rate = {}
    for cond in ("snow", "clear"):
        n = occ = 0.0
        for seq in sorted(p for p in (d / cond).iterdir() if p.is_dir()):
            k, r = boreas_occupied_rate(seq, field, h, max_frames)
            n, occ = n + k, occ + k * r
        rate[cond] = (int(n), occ / n if n else 0.0)
    extra = rate["snow"][1] - rate["clear"][1]
    return Result("pass" if extra < 0.01 else "fail",
                  f"field occupied in {100 * rate['snow'][1]:.1f} % of {rate['snow'][0]} snow frames and "
                  f"{100 * rate['clear'][1]:.1f} % of {rate['clear'][0]} clear frames",
                  "A 4 m by 3.8 m field ahead of the car at 0.5 m above the road. Traffic ahead occupies it in both "
                  "conditions; the difference is what snow adds. Target: under 1 percentage point. An automotive "
                  "lidar on a car roof, not a safety scanner on a loader; people are not labelled in these "
                  "sequences, so missed people are not measured.")


def cfar_detects(img: np.ndarray, cx: float, cy: float, half: int = 4, guard_px: int = 8, train_px: int = 20,
                 factor: float = 2.5) -> bool:
    """Cell-averaging CFAR on a radar image at one place: the strongest cell around (cx, cy) against
    the mean of a ring of training cells around it."""
    h, w = img.shape
    x, y = int(round(cx)), int(round(cy))
    if not (train_px <= x < w - train_px and train_px <= y < h - train_px):
        return False
    cell = img[y - half:y + half + 1, x - half:x + half + 1].max()
    win = img[y - train_px:y + train_px + 1, x - train_px:x + train_px + 1].astype(float)
    inner = img[y - guard_px:y + guard_px + 1, x - guard_px:x + guard_px + 1].astype(float)
    noise = (win.sum() - inner.sum()) / (win.size - inner.size)
    return bool(cell > factor * max(noise, 1.0))


RADIATE_M_PER_PX = 0.17361
PEOPLE = ("pedestrian", "group_of_pedestrians")


def radiate_people(seq: Path, max_frames: int):
    """(distance m, detected) for every labelled pedestrian in a RADIATE sequence's cartesian radar images."""
    from PIL import Image  # only needed when the dataset is present
    ann = json.loads((seq / "annotations" / "annotations.json").read_text())
    imgs = sorted((seq / "Navtech_Cartesian").glob("*.png"))[:max_frames]
    out = []
    for k, f in enumerate(imgs):
        img = None
        for obj in ann:
            if obj.get("class_name") not in PEOPLE or k >= len(obj["bboxes"]) or not obj["bboxes"][k]:
                continue
            x, y, w, h = obj["bboxes"][k]["position"]
            if img is None:
                img = np.asarray(Image.open(f).convert("L"))
            cx, cy = x + w / 2, y + h / 2
            dist = math.hypot(cx - img.shape[1] / 2, cy - img.shape[0] / 2) * RADIATE_M_PER_PX
            out.append((dist, cfar_detects(img, cx, cy)))
    return out


def v_dat_06(max_frames: int = 400) -> Result:
    d = _data("KAUHA_DATA_RADIATE")
    if d is None:
        return Result("needs data", "", "Set KAUHA_DATA_RADIATE to a folder of RADIATE sequences (each with "
                      "Navtech_Cartesian/*.png and annotations/annotations.json); fog and snow sequences are the "
                      "ones of interest. Needs Pillow.")
    seqs = [p for p in sorted(d.iterdir()) if (p / "annotations" / "annotations.json").exists()]
    people = [x for s_ in seqs for x in radiate_people(s_, max_frames)]
    near = [det for dist, det in people if dist <= 9.0]
    far = [det for dist, det in people if 9.0 < dist <= 30.0]
    if not near:
        return Result("open", f"{len(people)} labelled pedestrians, none within 9 m",
                      "The sequences given hold no pedestrian inside the radar field range; add sequences that do.")
    r_near, r_far = sum(near) / len(near), (sum(far) / len(far) if far else float("nan"))
    return Result("pass" if r_near >= 0.9 else "fail",
                  f"radar detects {100 * r_near:.0f} % of {len(near)} labelled pedestrians within 9 m and "
                  f"{100 * r_far:.0f} % of {len(far)} between 9 and 30 m ({len(seqs)} sequences)",
                  "Cell-averaging CFAR on the 360 degree scanning radar image at each labelled pedestrian. Target: "
                  "90 % within the 9 m the radar fields use. A Navtech imaging radar, not the safety radar; it "
                  "shows whether people stay visible to radar in fog and snow, not that the rated device detects them.")


def _not_built(what: str):
    return lambda: Result("open", "not built", what)


CHECKS = {
    "V-STD-01": v_std_01, "V-STD-02": v_std_02, "V-STD-03": v_std_03, "V-STD-04": v_std_04, "V-STD-05": v_std_05,
    "V-STD-06": v_std_06, "V-STD-07": v_std_07, "V-PUB-01": v_pub_01, "V-PUB-02": v_pub_02, "V-PUB-03": v_pub_03,
    "V-PUB-04": v_pub_04, "V-DAT-01": v_dat_01, "V-DAT-02": v_dat_02, "V-DAT-03": v_dat_03, "V-DAT-04": v_dat_04,
    "V-DAT-05": v_dat_05, "V-DAT-06": v_dat_06,
    "V-DAT-07": _not_built("The benchmark's download page and file format could not be read from the build "
                           "environment. Needs the format description, then the EKF run on its logged GNSS and "
                           "IMU instead of the twin's."),
}
