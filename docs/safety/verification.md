# Formal verification of the safety PLC logic

The safety requirements are proved, not only tested, on the code that runs.

## Method

`kauha/safety/core.py` holds the PLC's step function. It is written once and
executed in two ways:

| Evaluation | Ops object | Used by |
|---|---|---|
| Concrete (floats, ints, bools) | `core.CONCRETE` | `SafetyPLC`, the shift simulation, the SIL processes, every test |
| Symbolic (z3 terms) | `kauha.verify.symbolic.Z3Ops` | `kauha.verify.prove` |

All branching in `step` goes through the ops object (`O.If`, `O.And`,
`O.Min`, ...). Run on symbolic memory and inputs, the function returns one
formula for a PLC cycle over **every input and every memory state**.

The proof is in two parts:

1. **An inductive invariant `INV`.**
   - It has three parts:
     - "stopped" holds exactly when a cause is latched;
     - "emergency stop" holds exactly when an e-stop cause is latched;
     - the speed envelope is never below the speed limit.
   - It is proved to hold at power-up and to be preserved by every cycle.
   - So it holds in every reachable state.
2. **30 requirement properties.**
   - Each is proved for every input and every memory state that satisfies
     `INV`.
   - The search is not limited to states some test happened to reach.
   - "Proved" means z3 showed the negation unsatisfiable. A failure comes
     back as a concrete counterexample: a memory state and inputs.

Each property is proved in four configurations: dry and winter braking budgets,
each with and without automatic restart (SR-11). That is 120 proofs, plus the
invariant, in about 30 s. They run in CI (`tests/verify/test_proofs.py`) and
are traced to their SR in the traceability matrix.

`tests/verify/test_ops_agree.py` checks the bridge between the two
evaluations. It substitutes random concrete states into the symbolic formula
and requires the same outputs as the concrete step.

```bash
python -m kauha.verify.prove      # writes results/verification.json, prints any counterexample
```

## What the proofs found

The first proof run returned counterexamples for five properties. The causes
were four defects in logic that had passed every unit test, property-based test
and 15 simulated shifts. Each is fixed, and each now has a regression test
named `test_found_by_proof_*`.

| Property | Counterexample | Effect | Fix |
|---|---|---|---|
| SR-04b: speed within what the selected field stops from | 3.0–3.2 m/s (dry) on the 3.0 m/s band | Sustained. The overspeed tolerance let the machine run up to 0.2 m/s faster than the top-band field was sized for. At 3.2 m/s it needs 4.05 m to stop; the field was 3.71 m. Winter: 4.52 m needed against 4.0 m. | The top band uses the full 4 m scanner range. The PLC trips at once if the speed exceeds what the selected field protects. The speed limit is capped at the protected speed less the tolerance: 2.97 m/s dry, 2.41 m/s winter. |
| SR-02a/b: a violated front or rear field stops the machine outside recovery | Front and rear in recovery, a side stop, then a reset | One cycle (25 ms) at standstill with the brakes released while the front and rear fields were occupied and no longer in recovery. The reset *replaced* the recovery set instead of adding to it. | Recovery fields are added to the set (union). |
| SR-10b: muting longer than 25 s stops the machine | Reset pressed while digging continues past 25 s | The muting timeout was cleared by the reset. Muting carried on and tripped again on the next cycle, so the brakes were released for one cycle per reset. | A muting timeout counts as a standing cause, blocking reset while the dig request continues. A cause latched in a cycle can't be reset in the same cycle. |
| SR-12e: creep speed limit during recovery | The cycle in which recovery starts | The speed-limit output was still the full travel limit (3.0 m/s) for one cycle. The envelope monitor did not follow it, but the autonomy saw it. | The creep limit is applied after the reset logic. |

The SR-04b defect is the important one. It is not a corner case: the safety
function relied on the non-safety autonomy never overshooting 3.0 m/s. The
other three are one-cycle effects at standstill, and reaching them needs
unusual input sequences. They are still logic defects of the kind a safety
review is meant to catch.

## What is proved

The current list, with the text of each property, is generated into
`results/verification.json` and shown in the safety case. In short:

| Property | Statement |
|---|---|
| OUT-1 | A stop always cuts brakes, implement and speed limit |
| SR-01a/b | E-stop gives an emergency stop in the same cycle; never cleared without a new reset |
| SR-02a/b/c/d | Front, rear and side fields and scanner faults stop the machine in the same cycle (except the field under supervised recovery) |
| SR-03 | Warning hold caps speed at 1 m/s |
| SR-04a/b | Speed limit within the scanner cap; measured speed within the selected field's protected speed |
| SR-05, SR-06 | Link loss and watchdog stop the machine |
| SR-07a/b | Geofence and invalid-position timeout stop the machine |
| SR-08 | Speed-channel disagreement stops the machine |
| SR-09 | Resets only at standstill |
| SR-10a/b | Dig set only when requested, in zone, valid position, within speed; muting over 25 s stops |
| SR-11 | Only self-clearing causes restart without a supervisor |
| SR-12a–e | Recovery: implement off, motion toward the field trips, contact band trips, TELEOP only, creep limit |
| SR-13 | Every stop and every reset is logged |
| SR-14a/b | No dock set with the cab door open; door opening while docking stops |
| SR-16a/b/c | Person between the front tyres and the bucket (near and mid fields), under the raised bucket, or beside the front frame while reversing stops travel and holds the boom; recovery only backs away |

## What the adversarial search found next

The proofs show that the logic does what the requirements say. They cannot
show that the requirements are enough. An adversarial search over pedestrian
behaviour (`kauha/adversary`) answered that question for the integrated system:

- With the site rules kept, CMA-ES drove a person into contact (−0.25 m) with
  no protective stop at all. Random search with the same budget got no closer
  than 0.23 m.
- The cause was the area under and behind the bucket. After dumping, the loader
  reverses with the bucket raised, then lowers it on the way. A person who
  walks in front of it stands where the bucket comes down. No field covered
  that ground: the front fields start at the bucket's reach, and the
  front-frame side fields were only used in forward sets.
- New requirement SR-16 adds under-bucket fields, front-side fields while
  reversing and a two-channel boom-angle sensor. It was then proved
  (SR-16a/b/c) and searched again. Results: `docs/results/README.md`.
- The same search on the load-and-carry site found one more contact (−0.19 m).
  A person stood in the gap between the near field (which ends at 2.8 m) and
  the lowered bucket's back edge (3.57 m or more on level ground). With the
  boom low, the far field is not used, because at scan height it contains the
  bucket itself. So nothing covered the gap, the stop cleared, and the machine
  reversed with the bucket's back edge into the person. SR-16 now adds a mid
  field (2.8–3.35 m), watched at every boom position except in the dig zone and
  beside a docked truck. SR-16a was re-proved with it.

## What is not proved

These are the limits of the evidence, stated plainly:

- **Real numbers, not IEEE floats.** z3 reasons over exact reals. Rounding
  matters only at exact boundaries such as `|v| == band top`. The protected
  speeds are computed so that a band's field protects its own top exactly, and
  `test_ops_agree` compares both evaluations on random states.
- **One cycle at a time.**
  - The properties hold for every cycle from every reachable state.
  - Timing requirements that span cycles are proved only where the timer lives
    in memory (SR-07b, SR-08, SR-10b).
  - End-to-end stopping distance is checked by simulation and the SIL tests.
    That covers scanner response, brakes and vehicle motion.
- **Field geometry.** The proofs do not check that a field polygon covers the
  swept path and the stopping distance. They take each set's protected speed as
  given. Geometry is checked by sampled tests
  (`test_field_covers_swept_path_for_any_articulation`) and by the adversarial
  search.
- **The inputs.** The proofs assume the scanners, speed sensors and e-stops
  report the truth. That rests on certified devices and two-channel wiring, as
  in ISO 13849.
- **The toolchain.** This is a reference implementation in Python. A certified
  safety PLC needs the logic re-implemented in its own toolchain. The
  properties here then become the specification for that implementation's
  verification. z3 is not a qualified tool in the IEC 61508 sense.
