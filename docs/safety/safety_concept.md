# Safety concept

## 1. Architecture: two channels that do not share failure modes

| | Autonomy channel | Safety channel |
|---|---|---|
| Purpose | Productivity: plan, dig, load, avoid people early | Protection: stop before harm |
| Hardware | Jetson-class computer, lidar, radar, cameras, RTK GNSS | Safety PLC, safety scanners, two-channel speed sensor, safety GNSS, e-stops, relays |
| Software | Python/C++, learned components (bucket policy, person detector) | Deterministic logic, no learned components, certified toolchain |
| Failure response | Degrade (slow down), hold, ask the remote operator | Hardwired stop (brake, neutral, pilot cut-off) |

The split follows the EU Machinery Regulation (EU) 2023/1230, which applies
from 20 January 2027. Under it, safety components with self-evolving (learned)
behaviour need third-party conformity assessment (Annex I, Part A). Keeping
every safety function deterministic and outside the learned components keeps
the safety channel in the normal conformity route.

- The learned bucket policy and the person detector improve productivity
  and comfort. They are never relied on for protection.
- The autonomy channel slows down for people it sees. That reduces how often
  the safety channel has to act, but the safety channel never relies on it.
- The autonomy channel also limits its own speed so that mapped objects
  (stockpiles, the truck, walls) stay outside the field the PLC will select
  (AR-04). That removes most nuisance stops without touching the safety logic.

## 2. Hazards addressed and how

| Hazard | Primary measure | Supporting measures |
|---|---|---|
| Person struck while travelling | Speed-banded protective fields (SR-02), with the speed cap from the scanner range (SR-04) | Warning fields (SR-03), camera/lidar person slowdown, beacon and horn |
| Person struck while reversing | Rear fields (SR-02). Dig and dock sets are never used in reverse (SR-10). | Rear camera |
| Person in the pile face during digging | Dig zone is access-controlled. The front field is muted only inside the zone, at 1 m/s or less, for 25 s at most (SR-10). | Lidar/camera slowdown before entering the zone. Side fields stay active around the cab and articulation. |
| Person between the truck and the loader | Dock field covers the ground between the tyres and the truck side (SR-10). Dock zone speed cap 0.6 m/s. Truck cab-door interlock (SR-14). | Truck drivers stay in the cab (site rule). The demo scenario tests a driver who leaves the cab. |
| Person under the raised bucket, or between the front tyres and the bucket | Under-bucket fields and front-side fields while reversing, with the boom held on a stop; two-channel boom-angle sensor (SR-16) | Found by the adversarial search; regression scenario in `tests/adversary` |
| Crushing at the articulation joint; person beside the frames | Side fields on both frames while moving (SR-02). Teleop recovery from a side stop is refused while the contact band beside the frame is occupied (SR-12). | Site rule: no approach on foot in AUTO |
| Machine leaves the work area | Geofence on the independent safety position (SR-07) | Autonomy map |
| Autonomy computer fails or hangs | PLC watchdog (SR-06), gateway timeout | Hardwired stop path |
| Loss of remote supervision | Heartbeat monitor (SR-05) | Separate safety radio for the e-stop |
| Sensor fault (speed) | Two-channel plausibility (SR-08) | — |
| Corrupted, replayed or injected CAN frames | E2E counter + CRC with data ID; gateway timeouts | Gateway filters OEM messages (cybersecurity) |
| Winter braking | Winter stop budget lowers the speed cap (2.61 m/s) | Measured brake tests each season |

## 3. Residual risks to state openly

1. **Muting in the dig zone depends on an organisational measure.** People
   must be kept out of the dig zone. On site, back this with physical barriers
   or zone access control, for example tags or light curtains at zone entries.
2. **Scanner coverage close to the bucket** has a blind volume at ground level
   (see `docs/hardware/sensor_mounting.md`). It must be measured on the
   machine.
3. **The 4 m protective range caps speed at about 3 m/s.** Faster autonomous
   travel would need a longer-range safety-rated sensor, and none is chosen
   yet.
4. **Automatic restart after a field stop (SR-11)** is a site decision. The
   demo enables it after 3 s of clear fields. A conservative site can disable
   it, at the cost of more supervisor interventions.
5. **People under the raised bucket during the final approach to the truck (found by the twin).**
   - The dock field covers the ground between the front tyres and the truck side.
     A truck driver who steps out and stands beside the truck was under the raised
     bucket for 1–2 s before the dock field reached them.
   - Mitigated for the truck driver by SR-14: a cab-door switch on the truck, read
     through the bay's safety radio. With the door open the PLC refuses the dock
     field set (so the loader cannot get close) and stops a docking loader. The
     shift KPI `person_under_raised_bucket_moving_s` fell to zero in every run.
   - Still open for anyone else on foot in the dock zone. The zone is
     access-controlled like the dig zone. A downward-looking safety sensor on
     the boom would close it fully. It is listed as a candidate, not designed.
6. **Field geometry errors are easy to make and hard to see.** The twin found two:
   - fields built at one nominal articulation did not cover the full sector;
   - at standstill the rear field was placed at the bucket's reach instead of
     the counterweight, leaving 3.4 m behind the machine uncovered until it
     started moving.
   - the side fields covered only the front frame, so a person beside the rear
     frame was unprotected while the machine moved.
   All three are fixed and covered by tests. The lesson for the real machine is that
   every field set must be verified with a test body, not just reviewed.
8. **Teleop recovery beside the machine (found by the twin).** Recovery motion
   (SR-12) used to be allowed in either direction after a side-field stop. A
   person standing right beside the front frame was brushed by the bucket corner
   when the operator reversed, because the bucket is wider than the frame. Recovery
   is now refused while the contact band is occupied. Two dependencies remain:
   - a person 0.4–0.6 m out is outside the band, and recovery at 0.3 m/s past
     them relies on the operator watching the side cameras;
   - inside the dig zone, and the dock zone with the cab door closed, the band
     cannot block recovery, because the pile flank or the truck would block it for
     good (the twin showed this deadlock). There, protection rests on the zone
     access control, as for muting (item 1).
7. **Substantial modification.** Retrofitting autonomy to a machine already in
   service is very likely a substantial modification. Whoever does it takes on
   the manufacturer's obligations for the modified machine: conformity
   assessment, technical file, CE marking.

9. **Under-bucket coverage depends on the zones too.** Inside the dig zone the
   pile fills the under-bucket fields, and beside a docked truck the truck fills
   the far one. There the fields are not used, and the zone access control (A1)
   carries the risk. The adversarial search with the zone rules broken measures
   how much (`results/adversary_rules-broken.json`).
10. **Proofs cover the logic, not the requirements.** The z3 proofs show the
    PLC does what SR-01…SR-16 say (`verification.md`). Whether the requirements
    are enough is a different question. The adversarial search is the evidence
    for that, and it is only as good as its search space: one pedestrian,
    routes of two legs, dust. Two people working together, vehicles other than
    the truck, and sensor faults during the search are not yet covered.

## 4. Standards map

| Topic | Standard | Where it shows up here |
|---|---|---|
| Risk assessment | ISO 12100 | Section 2 (preliminary) |
| Safety-related control systems | ISO 13849-1/-2 | PL targets, Cat. 3 wiring, SR tests |
| Earth-moving machinery functional safety | ISO 19014-1…-4 | MPLr determination (to do), Annex H for wheel loaders |
| Autonomous earth-moving machinery | ISO 17757 | Mode management, remote supervision, site zoning |
| Remote operation | ISO 15817 | Teleop mode, supervisor reset, latency budget |
| Object detection systems | ISO 16001 | Field verification, person detection |
| Outdoor safety sensors | IEC 62998 | Scanner selection |
| Machinery Regulation | (EU) 2023/1230 | Section 1, residual risk 5, cybersecurity in the gateway |

## Open findings from external validation

Checks against standards and published attack classes (`docs/results/validation.md`)
found three limits the twin had not shown. None is closed yet.

1. **Scanner range.** The protective fields are measured from the bucket's front
   edge, but the front scanners sit about 3 m behind it. The far edge of the
   longest field is up to 8.1 m from the nearest scanner, beyond the 4 m protective
   range the speed cap (SR-04) assumes. The twin's scanners see 15 m, so no shift
   or search exposed it. Options: a rated sensor with about 9 m protective range, a
   mounting further forward, or a speed cap recomputed from the scanner position.
   The 1:8 testbed is not affected in the same way (its scanner sits at the frame
   nose with a 3 m range against 0.5 m fields), so it cannot be used to argue this
   point for the full machine.
2. **Person lying on the ground.** A horizontal scan plane at 0.5 m passes over
   the 70 mm lying test piece of ISO 3691-4. Options: a second low scan plane, a
   rated 3D sensor, or a stated residual risk covered by access control (A1).
3. **The bus is not authenticated.** CRC and alive counter detect faults, not
   attackers. A node on the bus that computes the CRC can make the gateway release
   while the PLC reports a stop (the PLC's relays still cut power), and a forged
   RemoteHeartbeat could hide a lost link or a pressed remote e-stop. Needed before
   any use outside a closed test area: message authentication (for example SecOC
   with a CMAC) or a physically separate safety segment, and the remote e-stop on a
   hardwired or authenticated channel.
