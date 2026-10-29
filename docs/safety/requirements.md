# Safety and autonomy requirements

Tests reference these IDs through `@pytest.mark.req("SR-xx")`. Running
`KAUHA_TRACE=results/traceability.json pytest` writes the traceability matrix
(requirement → tests → outcome).

PL targets are design targets taken from a preliminary risk assessment. They
are not certified values. The final allocation needs a full risk assessment
of the machine and site (ISO 12100) and verification to ISO 13849-1/-2. For
earth-moving machinery the ISO 19014 series gives MPLr. Work out the MPLr for
each function, using its Annex H for wheel loaders.

## Safety functions (independent channel, safety PLC)

| ID | Requirement | PL target | Implementation | Verification |
|---|---|---|---|---|
| SR-01 | A machine or remote e-stop shall bring the machine to an emergency stop and latch it. Reset needs the e-stop released, a supervisor reset and standstill. | d | `plc.py` SR-01 block; hardwired relays | `tests/safety/test_plc.py::test_sr01_*`, property `test_estop_always_results_in_emergency_stop` |
| SR-02 | An object in the protective field of the active direction (or the side fields while moving) shall cause a protective stop within one PLC cycle of the scanner output. Each field shall cover the swept path over the stopping distance at the top speed of its band, for every articulation angle in its sector. | d | Scanner field sets as convex envelopes over 5 articulation sectors (`fields.py`), PLC trip | `test_sr02_*`, `test_protective_field_covers_stopping_distance`, `test_field_covers_swept_path_for_any_articulation`, closed-loop `tests/scenario/test_person_intrusion.py` |
| SR-03 | An object in the warning field shall limit speed to 1.0 m/s without stopping. The limit holds for 2 s after the field clears. | b | PLC warning logic | `test_sr03_*` |
| SR-04 | Speed shall be monitored against the active limit. After a limit drops, the speed shall follow a 0.8 m/s² ramp, starting after a 0.3 s reaction time. The autonomous speed limit shall not exceed the speed the protective field range can protect for the current braking budget (dry or winter). | d | SS1-r-style envelope; `field_speed_cap` | `test_sr04_*`, property `test_speed_limit_never_exceeds_field_cap`, `test_stops_before_person_winter` |
| SR-05 | Loss of the remote supervision heartbeat for more than 500 ms in AUTO or TELEOP shall cause a protective stop. Reset needs a supervisor. | c | PLC heartbeat monitor | `test_sr05_*` |
| SR-06 | Loss of autonomy commands for more than 200 ms in AUTO or TELEOP shall cause a protective stop (watchdog). | c | PLC watchdog; gateway brakes at 150 ms | `test_sr06_*`, `tests/hil/test_virtual_bus.py` |
| SR-07 | In AUTO the machine shall stay inside the operating area. The check accounts for the safety position's uncertainty bound. An invalid position for more than 2 s shall cause a stop. | c | `geofence.py`, PLC | `test_sr07_*` |
| SR-08 | Disagreement between the two speed channels of more than 0.3 m/s for more than 0.2 s shall cause a stop. | d | PLC plausibility check | `test_sr08_*` |
| SR-09 | No reset shall be accepted unless the machine is at standstill. | d | PLC reset logic | `test_sr09_*` |
| SR-10 | The dig (muted front) and dock (short front) field sets shall be granted only when the machine is inside the matching zone, at or below its speed cap (1.0 / 0.6 m/s), and not reversing. Muting shall be limited to 25 s. | d | `select_field_set`, PLC | `test_sr10_*` |
| SR-11 | After a field-caused stop, automatic restart is allowed only if the site risk assessment permits it. It needs all fields clear for 3 s and no other latched cause. | c | PLC reset logic, `auto_restart_clear_s` | `test_sr11_*` |
| SR-12 | In TELEOP, a supervisor reset with an occupied protective field shall enter recovery. The speed limit is 0.5 m/s (0.3 m/s for the side fields), motion toward the occupied field trips again, and boom/bucket motion is disabled until the field clears. | d | PLC recovery logic | `test_sr12_*` |
| SR-13 | Every stop, reset and recovery shall be recorded in a tamper-evident log. | — | `logger.py` hash chain | `test_sr13_*`, `test_log_detects_tampering`, scenario `test_safety_log_intact` |

## Autonomy requirements (non-safety)

| ID | Requirement | Verification |
|---|---|---|
| AR-01 | Path tracking: forward cross-track error < 0.25 m. Reverse arcs on the rear axle < 0.45 m. | `tests/autonomy/test_autonomy.py` |
| AR-02 | Localisation stays below 0.3 m error through a 60 s GNSS outage when ≥ 3 survey posts are visible. Dead reckoning alone shall report growing uncertainty. | `tests/autonomy/test_autonomy.py` |
| AR-03 | Complete truck-loading V-cycles with fill factor 0.8–1.15 in the integrated twin. | `tests/scenario/test_shift_short.py` |

## What these tests do not prove

- **They do not prove the safety PLC hardware or toolchain.** The logic here
  is a Python reference model. It must be re-implemented in the certified
  environment and the same tests ported to it (HIL-01..10).
- **They do not prove the scanners detect people in real conditions.** The
  twin ray-casts ideal circles and segments with a dust model. Real
  detection depends on reflectivity, posture, snow and window contamination.
  Commissioning tests use a standardised test body.
- **They do not prove the braking numbers.** The `StopBudget` values are
  assumptions until measured on the machine, dry and in winter.
