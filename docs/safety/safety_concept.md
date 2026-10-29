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

## 2. Hazards addressed and how

| Hazard | Primary measure | Supporting measures |
|---|---|---|
| Person struck while travelling | Speed-banded protective fields (SR-02), with the speed cap from the scanner range (SR-04) | Warning fields (SR-03), camera/lidar person slowdown, beacon and horn |
| Person struck while reversing | Rear fields (SR-02). Dig and dock sets are never used in reverse (SR-10). | Rear camera |
| Person in the pile face during digging | Dig zone is access-controlled. The front field is muted only inside the zone, at 1 m/s or less, for 25 s at most (SR-10). | Lidar/camera slowdown before entering the zone. Side fields stay active around the cab and articulation. |
| Person between the truck and the loader | Dock field covers the ground between the tyres and the truck side (SR-10). Dock zone speed cap 0.6 m/s. | Truck drivers stay in the cab (site rule). The demo scenario tests a driver who leaves the cab. |
| Crushing at the articulation joint | Side fields while moving (SR-02) | Site rule: no approach on foot in AUTO |
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
   - A truck driver who steps out and stands beside the truck is under the raised bucket for about 1–2 s before the dock field reaches them, while the loader creeps in at 0.55 m/s.
   - The bucket does not touch them, because its bottom is above 2.5 m, but this is the "nobody under a raised load" hazard.
   - The demo scenario reproduces it on purpose, and the KPI `person_under_raised_bucket_moving_s` measures it.
   - Candidate fixes, both to be evaluated:
     - a downward-looking safety sensor on the boom;
     - a site rule enforced by a cab-door switch on the truck, which stops the loader when a truck door opens.
6. **Substantial modification.** Retrofitting autonomy to a machine already in
   service is very likely a substantial modification. Whoever does it takes on
   the manufacturer's obligations for the modified machine: conformity
   assessment, technical file, CE marking.

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
