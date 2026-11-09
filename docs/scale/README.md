# KAUHA 1:8 testbed: build pack

A 1:8 articulated loader built to test the KAUHA stack on a real machine before
the full-size retrofit. It carries the same safety logic, the same CAN messages
and the same ROS 2 interface as the digital twin, at Froude-scaled speeds and
distances. Everything here is generated from the repository's single sources and
checked in CI.

| What | Where | Made by |
|---|---|---|
| Design basis: scaling, references, decisions | [design_basis.md](design_basis.md) | |
| Mechanical design: STEP per part, assembly STEP, DXF for laser cutting, bucket flat pattern, STL for printed parts, drawings PDF, BOM | `hardware/scale/cad/export/` | `python -m hardware.scale.cad.scale_cad` |
| Actuator linkages (lift, bucket four-bar, steering) | `hardware/scale/generated/linkage.json` | `python -m kauha.scale.linkage` |
| Scaled safety profiles and field sets | `hardware/scale/generated/` | `python -m kauha.scale.profile` |
| Safety PLC program (CODESYS, IEC 61131-3 ST) | `plc/codesys/`, [plc.md](plc.md) | hand-written, tested against the reference |
| Gateway and sensor-node firmware (Teensy) | `firmware/` | shared code host-tested |
| Wiring: power, stop chain, CAN | [wiring.md](wiring.md), `hardware/scale/wiring/kauha_scale8_wiring.pdf` | `python -m hardware.scale.wiring.wiring_diagram` |
| Commissioning and test plan | [test_plan.md](test_plan.md) | |

## What is tested before anything is built

- **PLC program equals the proved logic.** `plc/codesys/src/FB_SafetyCore.st` is
  compiled with matiec and run against `kauha/safety/core.py` (the code the 120 z3
  proofs cover) for 200 000 cycles per profile in CI, plus scripted edge cases.
  Every output and every memory value must be bit-identical; four seeded one-token
  faults are each detected. The CAN glue (`FB_SafetyNode`) is tested in closed
  loop with frames from the Python codec: boot, reset, link loss, corrupt frames,
  wrong field-set echo, e-stop, zone loss.
- **Firmware equals the reference gateway.** The C gateway core decides exactly as
  `kauha/can/gateway.py` on 6000 random frame cycles with corrupted, repeated and
  missing frames. CRC and end-to-end checks are bit-identical. Wheel speeds follow
  the twin's articulated kinematics.
- **The machine fits together.** The CAD sweeps 30 poses (full articulation, boom
  and bucket extremes, rear-axle oscillation) for interference, keeps every
  actuator inside its stroke, keeps 25 mm ground clearance, and computes the static
  tipping load (ISO 14397-1 method).

## Order of work

1. Read [design_basis.md](design_basis.md) and the BOM; order the long-lead parts
   (hub motors, ODrives, actuators, safety relay).
2. Laser-cut the plates from the DXFs, bend the bucket, print the brackets.
3. Build the bench first: PLC, safety relay, relays, gateway, sensor node and CAN
   on a board, wheels off the ground. Run [test_plan.md](test_plan.md) T0-T2.
4. Assemble the machine; T3 measures the stopping distances. Put the measured
   values into `kauha/scale/profile.py`, regenerate, rerun the PLC tests, reload.
5. T4-T6: field checks with test pieces, teleop, then the autonomous V-cycle.

## Limits

- The Raspberry Pi running CODESYS is not a safety controller. On the testbed the
  hardwired chain (e-stops, PNOZ s4, relays with dynamic braking) is what stops the
  machine; the PLC repeats the decision in series. No person enters the test area
  while the machine is energised (test plan, rule 1).
- In the evaluator profile a hobby lidar and software stand in for the safety
  scanners. It is for testing the logic, not for protecting people.
- Soil does not scale (design basis, section 4): bucket-filling forces on the
  testbed are not those of the full machine.
- Several numbers are planning values until commissioning measures them:
  stopping deceleration, relay drop-out, potentiometer calibration, hub-motor Hall
  count. The test plan says where each one is measured and where it goes.
