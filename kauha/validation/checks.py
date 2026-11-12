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
    ours = v * (b.actuator_latency_s + b.brake_buildup_s) + v * v / (2 * b.decel_m_s2)
    candidate = kmh ** 2 / 150 + 0.2 * (kmh + 5)
    verdict = "meet" if ours <= candidate else "miss"
    return Result("open", f"twin stops from {kmh:.1f} km/h in {ours:.1f} m",
                  f"The limit formula is unreadable in the free sample. A commonly quoted form "
                  f"(v2/150 + 0.2(v + 5), v in km/h) would give {candidate:.1f} m, which the twin's assumed braking "
                  f"would {verdict}. Confirm the formula in the purchased standard, then measure the real brake.")


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
    # how far the protective fields reach from the scanners that must watch them
    from kauha.safety.fields import SCANNER_PROTECTIVE_RANGE_M
    from kauha.sim.sensors import FRONT_MOUNTS
    reach = max(min(math.hypot(x - mt.x, y - mt.y) for mt in FRONT_MOUNTS)
                for fs in sets.values() if fs.direction > 0 for x, y in fs.protective)
    range_note = (f" The fields are measured from the bucket's front edge, but the scanners sit about 3 m behind "
                  f"it: the far edge is up to {reach:.1f} m from the nearest scanner, beyond the "
                  f"{SCANNER_PROTECTIVE_RANGE_M:.0f} m protective range the speed cap assumes. The twin's scanners "
                  "see 15 m, so the shifts never showed it. The full machine needs a rated sensor with that range "
                  "(or one mounted further forward), or a lower speed cap.")
    if missed:
        return Result("fail", f"{total - len(missed)} of {total} test-piece positions detected",
                      "Missed at 1 degree resolution: " + "; ".join(missed[:6]) + "." + range_note)
    return Result("fail" if reach > SCANNER_PROTECTIVE_RANGE_M else "pass",
                  f"{total} of {total} test-piece positions detected in {len(sets)} field sets",
                  "2D twin: scanner resolution 1 degree, two adjacent beams needed, two scans." + range_note)


def v_std_05() -> Result:
    return Result("finding", "scan plane 0.5 m on the full machine, about 0.085 m on the 1:8 testbed",
                  "A horizontal scan plane at 0.5 m passes over a 70 mm lying test piece, so a person lying on the "
                  "ground is not detected by the safety scanners. ISO 3691-4 is written for indoor trucks with low "
                  "scanners. Options: a second, low scan plane; a rated 3D sensor; or stating it as a residual risk "
                  "covered by site access control. Not yet decided in the safety concept.")


def v_std_06() -> Result:
    rep = json.loads((REPO_ROOT / "hardware" / "scale" / "cad" / "export" / "report.json").read_text())
    st = rep["stability"]
    sand = rep["bucket_struck_volume_l"] * 1.6
    ok = st["rated_operating_load_kg"] >= sand
    return Result("pass" if ok else "fail",
                  f"rated load {st['rated_operating_load_kg']} kg, tipping load {st['tipping_load_full_turn_kg']} kg, "
                  f"bucket of sand {sand:.1f} kg", "From CAD mass properties; weigh the built machine to confirm.")


def v_std_07() -> Result:
    return Result("open", "not calculated", "Needs MTTFd, DC and CCF from the data sheets of the selected safety "
                  "relay, relays and scanners (SISTEMA or hand calculation).")


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
    return Result("open", "KAUHA median cycle 69 s (truck loading, twin)",
                  "The operator study reports cycle times only in plots. Needs the figures digitised or the pilot "
                  "site's own operator timing.")


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


def attack_fuzzing(n: int = 200_000, seed: int = 1) -> dict:
    """Random payloads on a protected identifier, between genuine frames."""
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


def attack_masquerade(cycles: int = 400) -> dict:
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


def attack_replay(n: int = 20_000, seed: int = 2) -> dict:
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
    measured = (f"fuzzing: {fz['accepted']} of {fz['forged']} random frames accepted ({100 * fz['rate']:.3f} %); "
                f"replay: {100 * rp['rate']:.1f} % accepted; masquerade with a computed CRC: gateway released in "
                f"{mq['gateway_released_while_plc_stopped']} of {mq['cycles']} cycles while the PLC said stop; "
                f"suspension: brake after {su['brake_after_s']} s")
    ok = fz["accepted"] == 0 and rp["accepted"] == 0 and mq["gateway_released_while_plc_stopped"] == 0
    return Result("pass" if ok else "fail", measured,
                  "The CRC and alive counter detect faults, not attackers: they use no secret, so anyone on the bus "
                  "can forge frames. The stop itself survives (the PLC cuts power through relays, not over CAN), but "
                  "a forged RemoteHeartbeat could hide a lost link or a pressed remote e-stop. The bus needs message "
                  "authentication (for example AUTOSAR SecOC with a CMAC) or physical separation of the safety "
                  "segment before it leaves a closed test area. Suspension is handled.")


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
    return {"frames": len(frames), "decoded_as_recorded": as_recorded, "accepted_remapped": accepted}


def v_dat_02() -> Result:
    d = _data("KAUHA_DATA_ROAD")
    if d is None:
        return Result("needs data", "", "Set KAUHA_DATA_ROAD to a folder of candump logs.")
    logs = sorted(d.rglob("*.log"))[:20]
    tot = {"frames": 0, "accepted_remapped": 0}
    for p in logs:
        r = replay_log_on_bus(parse_candump(p))
        tot["frames"] += r["frames"]
        tot["accepted_remapped"] += r["accepted_remapped"]
    ok = tot["accepted_remapped"] == 0
    return Result("pass" if ok else "fail", f"{tot['accepted_remapped']} of {tot['frames']} frames accepted "
                  f"({len(logs)} logs)", "Recorded traffic carries no KAUHA CRC, so acceptance is by chance only.")


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


def _not_built(what: str):
    return lambda: Result("open", "not built", what)


CHECKS = {
    "V-STD-01": v_std_01, "V-STD-02": v_std_02, "V-STD-03": v_std_03, "V-STD-04": v_std_04, "V-STD-05": v_std_05,
    "V-STD-06": v_std_06, "V-STD-07": v_std_07, "V-PUB-01": v_pub_01, "V-PUB-02": v_pub_02, "V-PUB-03": v_pub_03,
    "V-PUB-04": v_pub_04, "V-DAT-01": v_dat_01, "V-DAT-02": v_dat_02, "V-DAT-03": v_dat_03, "V-DAT-04": v_dat_04,
    "V-DAT-05": _not_built("Needs the dataset's calibration and label format in hand before a loader is written."),
    "V-DAT-06": _not_built("Radar replay needs the radar's role fixed first (detection only, or part of a field)."),
    "V-DAT-07": _not_built("Needs the benchmark's file format and the EKF run from logged GNSS/IMU instead of the "
                           "twin."),
}
