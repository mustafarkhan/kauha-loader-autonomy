# KAUHA: autonomy retrofit for articulated wheel loaders

KAUHA turns a 14–18 t articulated wheel loader into a supervised autonomous
loader: truck loading, and load-and-carry from a stockpile to a fixed hopper. The retrofit kit adds sensors, a drive-by-wire gateway, an
autonomy computer and an independent safety channel. One remote supervisor
watches several machines and steps in by teleoperation when a machine asks
for help.

the similar concept is what i did for the government of Pakistan but for classified systems this is a simple take on that end of capability.

This repository contains everything that can be built and tested on this domain

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
| Formal verification | The safety PLC's step function is written once and run two ways: on floats in the machine, on z3 terms for proof. 30 requirement properties proved for every input and every reachable state, in four configurations, in CI | `kauha/safety/core.py`, `kauha/verify`, `docs/safety/verification.md` |
| Adversarial testing | CMA-ES searches pedestrian routes, timing and dust to minimise clearance to the moving machine, with the site rules kept or broken; found scenarios become regression tests | `kauha/adversary`, `tests/adversary` |
| Safety case | GSN assurance argument generated from the hazard list and the evidence (proofs, traced tests, shifts, search, SIL); CI fails if any claim loses its evidence | `kauha/assurance`, `docs/safety/hazards.yaml`, `docs/results/safety_case.html` |
| ROS 2 (Jazzy) | `kauha_ros` package: the twin as ROS 2 topics (TF, URDF, LaserScan per safety scanner, odometry, diagnostics, field markers), rosbag2 MCAP recording, and a CAN bridge so ROS autonomy drives the SIL machine while the safety PLC stays on CAN | `ros2/` |
| Load-and-carry | Hopper over a feeder, carry route, hopper dumping, KPIs against truck loading (`--task carry`) | `hardware/config/site_carry.yaml`, `docs/results/README.md` §4 |
| Concept CAD | Parametric CadQuery model of the 32-part kit on a simplified loader; STEP/STL per part, assembly STEP, GLB; checks that scanner origins match the twin and that nothing interferes over the joint ranges; interactive viewer | `hardware/cad`, `docs/results/kauha_cad.html` |
| Remote console | Clickable prototype of the supervisor's station on three recorded shifts: fleet, requests, 360° view, stop, e-stop, reset, task-space teleop, hash-chained log | `kauha/demo/console.py`, `docs/results/kauha_console.html` |
| Demonstration | 60-minute shift with people, link loss, an obstacle, dust, a GNSS outage, a material change, a computer freeze and a remote e-stop; KPIs, 2D and 3D replay | `kauha/demo`, `docs/results` |

## Quick start

```bash
pip install -e ".[dev]"            # core + tests
pytest -m "not slow"               # ~1 min
pytest -m slow                     # real-time multi-process SIL, ~1 min
python -m kauha.verify.prove       # prove the safety PLC properties (~30 s)
python -m kauha.assurance.case --check
python -m kauha.demo.shift --controller learned --minutes 60 --out results/shift_learned
python -m kauha.demo.shift --controller learned --minutes 60 --task carry --out results/shift_carry
pip install -e ".[adversary]" && python -m kauha.adversary.search --mode rules-kept --budget 160
```

ROS 2: see `ros2/README.md` (RoboStack environment, `colcon build`, launch files).

CAD (optional, about 150 MB of wheels):

```bash
pip install -e ".[cad]"
python -m hardware.cad.kauha_cad --check   # positions and interference only
python -m hardware.cad.kauha_cad && python -m hardware.cad.viewer
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
   measured braking decide the top protected speed: 3.17 m/s dry and 2.61 m/s
   winter with the current assumptions. The PLC grants 0.2 m/s less (2.97 and
   2.41 m/s), so that even the overspeed tolerance is covered by a field.
4. **Prove, then attack.** The safety PLC's step function is proved against its
   requirements for every input and state, and an optimiser then searches for
   scenarios in which the whole stack still hits someone. Both have found
   defects that every test had passed.

## Status and limits

This is a design and simulation baseline, for testing

- The soil model is a reduced FEE-style model. It has  been validated
  against measured bucket forces.
- Braking and scanner response times are after baseline tests.
- The Python safety logic is a reference model. The production version went into a PLC toolchain
- The 3D lidar is modelled at object level for 1st run then point clouds in next version
- The remote console is a prototype on recorded shifts, not a live link to the
  twin. 
- The CAD is concept geometry for layout are drawings and in version 2 or v2 you can download our cad and solidworks files too
- 
`docs/results/README.md` lists what the simulation results do and do not
show.


detector is trained with Ultralytics YOLO11 on the Construction-PPE dataset,
and both are AGPL-3.0. The trained weights (`models/ppe_yolo11n.pt`) are
therefore AGPL-3.0. A commercial product must either comply with the AGPL or
use a commercial licence or another detector.
