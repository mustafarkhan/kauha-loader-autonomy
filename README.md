# KAUHA: autonomy retrofit for articulated wheel loaders

KAUHA turns a 14–18 t articulated wheel loader into a supervised autonomous
truck loader. The retrofit kit adds sensors, a drive-by-wire gateway, an
autonomy computer and an independent safety channel. One remote supervisor
watches several machines and steps in by teleoperation when a machine asks
for help.

This repository contains everything that can be built and tested before a
physical machine is involved:

| Layer | What is here | Where |
|---|---|---|
| Hardware design | Candidate BOM, interface spec, wiring/architecture, sensor mounting, HIL bench spec, J1939 DBC | `docs/hardware`, `hardware/` |
| Safety channel | Deterministic safety PLC logic: speed-banded field sets, protective/warning fields, speed envelope, heartbeat, watchdog, geofence, dig/dock muting, supervised recovery, hash-chained event log | `kauha/safety` |
| Machine bus | DBC, encoder/decoder, end-to-end protection (counter + CRC-8 J1850), drive-by-wire gateway | `kauha/can` |
| Digital twin | Articulated loader dynamics, bucket–pile interaction, depleting stockpiles, trucks, people, ray-cast safety scanners, lidar/camera/GNSS/IMU models, dust, outages | `kauha/sim` |
| Autonomy | EKF localisation (GNSS + survey posts), pile/truck/people perception, V-cycle planner, articulated pure pursuit, field-aware speed limiting, supervisor, learned bucket filling | `kauha/autonomy` |
| Learning | PPO bucket-filling policy with domain randomisation; YOLO11n person/PPE detector fine-tuned on Construction-PPE | `kauha/train`, `models/` |
| Verification | Requirement-traced unit, property-based, closed-loop and end-to-end tests; CAN tests over python-can | `tests/`, `docs/safety/requirements.md` |
| Multi-process SIL | Machine, safety PLC, autonomy and remote station as separate processes on a shared CAN bus; fault injection by killing processes | `kauha/sil`, `tests/sil` |
| Demonstration | 60-minute shift with people, link loss, an obstacle, dust, a GNSS outage, a material change, a computer freeze and a remote e-stop; KPIs and replay | `kauha/demo/shift.py`, `docs/results` |

## Quick start

```bash
pip install -e ".[dev]"            # core + tests
pytest -m "not slow"               # ~1 min
pytest -m slow                     # real-time multi-process SIL, ~1 min
python -m kauha.demo.shift --controller learned --minutes 60 --out results/shift_learned
```

Training (optional, CPU is enough):

```bash
pip install -e ".[train]"
python -m kauha.train.tune_baseline            # fair rule-based baseline
python -m kauha.train.train_bucket --steps 2000000
python -m kauha.train.train_ppe --data-root /path/to/construction-ppe --epochs 15
```

## Design rules

1. **Protection never depends on learned components.** The bucket policy and
   the person detector make the machine productive and polite. The safety
   PLC, scanners and hardwired stops make it safe. See
   `docs/safety/safety_concept.md`.
2. **The autonomy computer requests and the safety PLC grants.** Muted (dig)
   and short (dock) field sets are granted only from the PLC's own zone and
   speed checks.
3. **Physics sets the speed limit.** The scanner's protective range and the
   measured braking decide the top autonomous speed: 3.17 m/s dry and
   2.61 m/s winter with the current assumptions.

## Status and limits

This is a design and simulation baseline, not a validated product.

- The soil model is a reduced FEE-style model. It has not been validated
  against measured bucket forces.
- Braking and scanner response times are assumptions to be measured.
- The Python safety logic is a reference model. The production version goes
  into a certified safety PLC toolchain.
- The 3D lidar is modelled at object level, not as point clouds.

`docs/results/README.md` lists what the simulation results do and do not
show.

## Licences

No licence has been chosen for the code yet (all rights reserved until the
owner picks one; decide this before publishing the repository). The person/PPE
detector is trained with Ultralytics YOLO11 on the Construction-PPE dataset,
and both are AGPL-3.0. The trained weights (`models/ppe_yolo11n.pt`) are
therefore AGPL-3.0. A commercial product must either comply with the AGPL or
use a commercial licence or another detector.
