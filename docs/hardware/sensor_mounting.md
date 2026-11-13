# Sensor mounting

The concept CAD (`hardware/cad/kauha_cad.py`, viewer `docs/results/kauha_cad.html`)
places every part below, and its check sweeps the joints for interference.

Coordinates are machine coordinates. The front-frame origin is the front axle
centre, with x forward. The rear-frame origin is the rear axle centre, with x
pointing backwards. Height is above ground.

| Sensor | Body | Position (x, y, z) m | Orientation | Why there |
|---|---|---|---|---|
| Front-left safety scanner | front | (2.0, +1.35, 0.5) | yaw +45°, 275° scan | Sees past the bucket edge, the left front corner, and back along the front frame (side field and contact band) |
| Front-right safety scanner | front | (2.0, −1.35, 0.5) | yaw −45°, 275° scan | Mirror of the above |
| Rear-left safety scanner | rear | (1.7, −1.2, 0.5) | yaw −45°, 275° scan | Behind the counterweight, and forward along the rear frame to the articulation joint |
| Rear-right safety scanner | rear | (1.7, +1.2, 0.5) | yaw +45°, 275° scan | Mirror of the above |
| Front safety radars (3) | front | (2.0, 0, about 1.8) and (2.0, ±1.0, about 1.8) | centre straight ahead, outer two yaw ±25° | The far part of the front protective field, 4 to 8.1 m from the sensors, over the lowered bucket. Rated 9 m within 40°, 5 m within 100°, 20° vertical |
| Rear safety radars (3) | rear | (1.7, 0, about 1.5) and (1.7, ±0.9, about 1.5) | centre straight back, outer two yaw ±25° | The far part of the rear protective field |
| 3D lidar | front (sensor mast) | (−0.4, 0, 3.4) | level | Over the bucket at carry height; sees the pile face and the truck body. On a mast on the front frame, so it turns with the bucket |
| 4D radar | front (frame nose) | (1.16, 0, 1.0) | forward | Dust fallback. Moved from (2.2, 0, 1.2) during CAD layout: there the boom's tilt cylinder struck it with the boom fully lowered |
| Cameras | rear (cab roof edges) | — | front, rear, left, right | Person detection and remote video |
| GNSS antennas | front (sensor mast crossbar) | (−0.6, ±0.8, 3.4) | — | 1.6 m baseline for heading, on the same frame as the lidar |
| Safety GNSS antenna | rear frame | (0.5, 0, 2.8) | — | Separate from the RTK antennas |

Turning each corner scanner 45° outward uses the 275° scan angle to look both
ahead/behind and along its own side of the frame. One scanner per corner then
covers the travel field, the side field and the contact band. The twin found
that with the earlier ±20° yaw the sides of the rear frame were not seen at all.

Each scanner holds 43 field sets (2 directions × 4 speed bands × 5 articulation
sectors, standstill, dig, dock). The front scanners monitor up to eight fields at
a time: protective, warning, side, contact band, front side, and the near, mid and
far under-bucket fields (SR-16). The rear scanners monitor four. The outdoorScan3 stores up to
128 fields and monitors eight at the same time (vendor data; confirm for the
ordered variant). The front scanners are therefore at that limit, with no spare
simultaneous field; a further field would need a second scanner pair or a
merged field.

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

## Why the radars are there

The front protective field starts at the bucket edge, 5.1 m ahead of the front axle,
and is up to 4 m long. The front scanners sit 3.1 m behind the bucket edge, so the
field ends up to 8.1 m from them. Their rated protective range is 4 m. Six safety
radars on one control unit (the product limit) cover the rest. `kauha/safety/coverage.py`
holds the rated coverage of every sensor and checks each field set against it; the
heights above are proposals, to be fixed on the machine so the radar looks over the
lowered bucket. Open points for the radar: confirmation for a moving outdoor machine,
response time, PFHd, and behaviour with the boom and bucket moving in its view.
