# Sensor mounting

Coordinates are machine coordinates. The front-frame origin is the front axle
centre, with x forward. The rear-frame origin is the rear axle centre, with x
pointing backwards. Height is above ground.

| Sensor | Body | Position (x, y, z) m | Orientation | Why there |
|---|---|---|---|---|
| Front-left safety scanner | front | (2.0, +1.35, 0.5) | yaw +45°, 275° scan | Sees past the bucket edge, the left front corner, and back along the front frame (side field and contact band) |
| Front-right safety scanner | front | (2.0, −1.35, 0.5) | yaw −45°, 275° scan | Mirror of the above |
| Rear-left safety scanner | rear | (1.7, −1.2, 0.5) | yaw −45°, 275° scan | Behind the counterweight, and forward along the rear frame to the articulation joint |
| Rear-right safety scanner | rear | (1.7, +1.2, 0.5) | yaw +45°, 275° scan | Mirror of the above |
| 3D lidar | front (cab roof) | (−0.4, 0, 3.4) | level | Over the bucket at carry height; sees the pile face and the truck body |
| 4D radar | front | (2.2, 0, 1.2) | forward | Behind the bucket's swept volume; dust fallback |
| Cameras | cab roof corners | — | front, rear, left, right | Person detection and remote video |
| GNSS antennas | cab roof | (−0.6, ±0.8, 3.5) | — | 1.6 m baseline for heading |
| Safety GNSS antenna | rear frame | (0.5, 0, 2.8) | — | Separate from the RTK antennas |

Turning each corner scanner 45° outward uses the 275° scan angle to look both
ahead/behind and along its own side of the frame. One scanner per corner then
covers the travel field, the side field and the contact band. The twin found
that with the earlier ±20° yaw the sides of the rear frame were not seen at all.

Each scanner holds 43 field sets (2 directions × 4 speed bands × 5 articulation
sectors, standstill, dig, dock). It monitors up to four fields at a time:
protective, warning, side and contact band. The outdoorScan3 stores up to
128 fields and monitors eight at the same time (vendor data; confirm for the
ordered variant).

## Known limitations to resolve on the real machine

- **Blind zone under and just ahead of a lowered bucket.**
  - At 0.5 m scanner height, a person crouching against the bucket is not
    guaranteed to be detected.
  - Mitigation: the approach to the face happens only inside the
    access-controlled dig zone. The field contour is set to the furthest bucket
    reach (5.1 m).
  - Verify with an ISO 5006-style visibility test and a test body.
- **The scanner plane meets rising ground.**
  - On ramps the protective field can see the ground ahead and trip.
  - Field sets for ramps, or a pitch-compensated mount, need site data.
- **Dust and snow on windows.**
  - The scanners report contamination, which maps to SCANNER_FAULT.
  - Window heaters and air knives are in the BOM.
  - The twin models airborne dust but not window contamination.
- **Bucket over the truck.**
  - In the dock field set the protective field covers only the ground between
    the front tyres and the truck side (2.0–2.8 m ahead of the front axle).
  - The truck must be parked so that its side is at least 2.8 m from the front
    axle at the dump pose. The planner docks at 3.1 m.
