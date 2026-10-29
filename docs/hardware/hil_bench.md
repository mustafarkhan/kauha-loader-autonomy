# Hardware-in-the-loop bench

Purpose: test the real safety PLC, gateway and autonomy computer against the
digital twin before anything moves. The same scenario files and pass criteria
as the software tests are used, so a result on the bench can be compared
directly with a result in CI.

## Bench layout

```
 ┌──────────────┐   CAN (KAUHA bus)    ┌───────────────┐
 │ Autonomy     │◄────────────────────►│ Gateway ECU   │◄──► CAN interface ◄──┐
 │ computer     │                      └───────────────┘     (OEM side)       │
 │ (Jetson)     │◄──── Ethernet (sim lidar/camera/GNSS) ◄──┐                  │
 └──────────────┘                                          │                  │
        ▲  CAN                                     ┌────────┴────────┐         │
        └──────────────────────────────────────────┤ Twin PC         ├─────────┘
 ┌──────────────┐  safe I/O (24 V)  ┌──────────┐   │ kauha.sim.*     │
 │ Safety PLC   │◄─────────────────►│ I/O sim  │◄──┤ (real-time, 40  │
 │ (target HW)  │                   │ (relays, │   │  Hz lockstep)   │
 └──────────────┘                   │  OSSDs)  │   └─────────────────┘
                                    └──────────┘
```

- The twin PC runs `kauha.sim` (loader, soil, scanners, people, events) in
  real time. It plays the OEM machine bus and the sensors.
- The I/O simulator turns the twin's scanner field results into OSSD signal
  pairs, and reads the PLC's relay and valve outputs back into the twin's brake
  and pilot-supply model.
- On CAN, `python-can` with `socketcan` replaces the `virtual` interface
  used in `tests/hil/test_virtual_bus.py`.

## Equipment

- Industrial PC with 4-channel CAN-FD (socketcan) and 16 DI / 16 DO at 24 V,
  for the OSSD simulation.
- Target safety PLC, gateway ECU and autonomy computer: the same parts as the
  BOM.
- Programmable power supply, so that low battery and cranking voltage
  can be tested.
- CAN fault injector. It must corrupt bits, drop frames, delay frames and
  short CAN_H/CAN_L.

## Test cases

| ID | Test | Pass criterion | Twin test it mirrors |
|---|---|---|---|
| HIL-01 | Person in protective field at each speed band, both directions, dry/winter budgets | Brake relay drops within 50 ms of OSSD; twin shows gap > 0 | `tests/scenario/test_person_intrusion.py` |
| HIL-02 | Autonomy computer power-cut / process kill | Gateway brakes ≤ 150 ms; PLC watchdog stop ≤ 200 ms | SR-06 tests, shift scenario at 48 min |
| HIL-03 | Radio link loss 0.5–5 s | Protective stop at 500 ms; reset only by supervisor | SR-05 tests, shift scenario at 12 min |
| HIL-04 | CAN corruption, replay, masquerade, bus-off | Frames rejected, brake within timeout, no unintended motion | `tests/can/test_can.py` |
| HIL-05 | Speed sensor disagreement (unplug one channel) | SPEED_DISCREPANCY stop ≤ 250 ms | SR-08 tests |
| HIL-06 | Dig-field request outside dig zone or above 1 m/s | Request refused | SR-10 tests |
| HIL-07 | Muting held > 25 s | MUTING_TIMEOUT stop | SR-10 tests |
| HIL-08 | Full 60-minute shift scenario in real time | KPIs within 10 % of the CI run; zero contacts | `kauha.demo.shift` |
| HIL-09 | Power interruption 50 ms / 500 ms | Safe state, and no restart without a reset | — |
| HIL-10 | Event log extraction after each test | Hash chain verifies | SR-13 |

## Exit criteria for moving to the machine

- All HIL tests pass on three consecutive days.
- The PLC program is verified against the requirements: SR tests are ported to
  the vendor's test harness, and a review against ISO 13849-2 validation is
  complete.
- Brake performance is measured on the target machine, dry and on ice/snow.
  The `StopBudget` values in `kauha/safety/stopping.py` are replaced by the
  measured values, and the field sets are regenerated.
