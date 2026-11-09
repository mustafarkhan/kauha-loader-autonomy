# 1:8 testbed: design basis

## 1. Why a scale machine, and why 1:8

Earthmoving automation has used scale models since the first research rigs:
Seward, Bradley and Bracewell built a 1/5-scale hydraulic backhoe to test control
strategies before risking a full-size machine, because a full machine moves at
about 3 m/s with tonnes of force [1]. Vehicle-dynamics research uses scaled
testbeds for the same reason and has worked out when their results transfer:
dynamic similarity holds when the dimensionless groups match, which for vehicles
dominated by gravity and inertia means Froude scaling [2, 3]. Model-scale vehicle
labs are now a standard way to test networked and autonomous driving functions [4].

KAUHA needs a machine that can run the real stack (safety PLC, CAN bus,
drive-by-wire gateway, ROS 2 autonomy, remote console) at speeds and distances
that keep the safety logic meaningful, small enough to build and test indoors in
days. 1:8 gives a 0.98 m long machine at about 30 kg (Froude) with a 0.5 m
protective field.

## 2. Scaling

Reference machine: `hardware/config/loader_medium.yaml` (15.5 t, 3.0 m3 bucket;
close to the Komatsu WA320-7 used in published sim-to-real work [5]).
Froude scaling with k = 8 (`kauha/scale/profile.py`):

| Quantity | Law | Full size | 1:8 testbed |
|---|---|---|---|
| Length | 1/k | lf = lr 1.6 m, track 2.1 m, boom 2.95 m | 0.20 m, 0.2625 m, 0.369 m |
| Speed | 1/sqrt(k) | auto limit 3.0 m/s | 1.06 m/s |
| Time | 1/sqrt(k) | muting limit 25 s | 8.8 s |
| Mass, force | 1/k^3 | 15.5 t, breakout 140 kN | 30.3 kg, 273 N |
| Torque | 1/k^4 | | |
| Acceleration, angles | 1 | brake 2.5 m/s2, articulation 40 deg | same |

Gravity, inertia and braking keep their ratios, so stopping distances and field
lengths scale like the geometry. Three things do not scale, and the testbed takes
them from its own hardware:

1. **Reaction times of real parts.** The SICK nanoScan3 responds in 70 ms [6], a
   10 Hz hobby lidar with a software evaluator in about 200 ms. The PLC still
   cycles at 25 ms. These go into the testbed's stop budget unscaled, which is why
   its field speed cap (1.07 m/s with nanoScan3, 0.89 m/s with the evaluator) is a
   little below the Froude-scaled 1.12 m/s.
2. **Communication timeouts** (heartbeat 0.5 s, watchdog 0.2 s) stay at their
   full-size values: the testbed uses the same links.
3. **Soil.** Cohesion and grain size do not follow Froude scaling, so
   bucket-filling forces are not similar. Published work on the WA320-7 shows even
   full-size simulators differ from the real machine by about 10 % in loaded mass
   and 11 % in forces [5]; a scale pile adds more. The testbed is for navigation,
   the V-cycle, the safety functions and the operator workflow, not soil forces.

The scaled profiles (field table, PLC parameters, 43 field sets each) are
generated in `hardware/scale/generated/` and loaded into the PLC as
`GVL_Config.st`. The proofs' consistency rules (`kauha.scale.profile.check`) hold
for every profile, and the test suite checks that every band's protective field
stops the machine from the speed it protects.

## 3. Mechanical design decisions

**Drive: four hub motors.** Independent wheel drive keeps the twin's no-scrub
articulated kinematics (each wheel at its own speed). A geared motor per wheel
does not fit: at 1:8 the space between the frame and the tyre is under 50 mm,
and transverse gearmotors (about 115 mm long) cross the centre line. 6.5 in
hoverboard hub motors put the motor inside the wheel, give far more torque than
the 160 N traction limit needs, have Hall sensors, and cost little. Tyre
radius 82.5 mm instead of the Froude 97.5 mm; the boom pivot height is kept.

**Stopping: dynamic braking through relays.** On a stop the safety relay drops
four 3-pole relays (K-DRIVE). They disconnect each hub motor from its ODrive and
short its phases through 0.47 ohm resistors, which brakes the machine without any
software. The deceleration is a planning value (2.0 m/s2) until test T3 measures it.

**Actuators: PA-14P linear actuators** with potentiometer feedback in place of the
hydraulic cylinders: two lift, one tilt and two steering, like the real machine.
Their retracted length (stroke + 140 mm) is the tightest constraint at this
scale, so `kauha/scale/linkage.py` searches anchor points for each actuator. Each
must stay inside its stroke over the full range with the largest worst-case moment
arm, then refine locally:

| Actuator | Stroke | Worst-case arm | Torque | Needed |
|---|---|---|---|---|
| Lift (x2) | 4 in | 48.8 mm | 65 N·m | 48 N·m, full bucket at carry reach |
| Tilt | 4 in | 32 mm in the dig range | 21.5 N·m | 8.3 N·m to hold a full bucket |
| Steering (x2) | 6 in | 26.2 mm | 35 N·m | 14.7 N·m, scaled standstill steering moment (assumed full size 60 kN·m) |

**Bucket linkage.** The testbed needs about 143 deg of bucket travel relative to
the boom. A single actuator on a lever gives well under 120 deg with usable force.
So the bucket uses the side-link plus H-link four-bar of excavator buckets,
mounted on the boom; it reaches the full range from one 4 in stroke. The
full-size Z-bar couples bucket and boom angles; the testbed keeps the twin's
convention of a bucket angle relative to the boom instead. Its breakout torque is
about half the Froude-scaled value, which is acceptable because soil forces are
not similar anyway.

**Structure.** 5 mm aluminium 5083 plates and 20x20 profile, bolted, no welding on
the frames (laser-cut parts can be ordered online and assembled in a day). The
bucket is a 1.5 mm steel sheet with four bends, with welded sides and edge. The
joint "waist" is narrow so that ±40 deg of articulation clears; the CAD sweep
found and removed six clashes there before release (front and rear plates,
cab and steering actuators, boom cross tube and tilt actuator, bucket back and
boom arms).

**Stability.** ISO 14397-1 rates a wheel loader's operating load at no more than
50 % of its static tipping load at full turn [7]. The CAD computes the tipping
load from the mass properties: at full turn the support triangle runs from the
front tyres to the rear-axle oscillation pivot. With the 11 kg steel counterweight the
testbed weighs 42 kg; the tipping load is 21.6 kg straight and 18.2 kg at full turn,
so the rated operating load is 9.1 kg. A struck bucket of sand (5.15 L) is 8.2 kg.
The testbed is heavier than the Froude 30 kg because of the hub motors; it is more
stable, and its stopping is measured, not assumed.

## 4. Safety architecture on the testbed

The structure is the full-size one (docs/safety/safety_concept.md), with stand-ins
stated plainly:

| Function | Full size | Testbed |
|---|---|---|
| Safety controller | certified safety PLC, PL d | CODESYS on a Raspberry Pi running the same logic (not rated) behind a PNOZ s4 safety relay (rated) |
| People detection | 4 outdoor safety scanners | 2 SICK nanoScan3 I/O (PL d, Type 3) [6], or for first tests 2 hobby lidars with `kauha/scale/field_eval.py` (not rated) |
| Stop | brakes + pilot cut-off, hardwired | dynamic braking + actuator power cut through relays, hardwired |
| Own speed / angles | PLC's own sensors | separate sensor node with its own supply |
| Geofence | safety GNSS | zone source frame from the supervisor station |

Driverless-truck practice (ISO 3691-4 [8]) is the closest guide for testing
personnel detection with test pieces; the test plan uses its approach of
verifying each field set with a test piece at the field edge. It never uses people.

## 5. Software reuse

The PLC runs `FB_SafetyCore`, a line-by-line port of `kauha/safety/core.py`, with
generated configuration lists for the full machine and both testbed profiles. The
port is not trusted on its own: matiec [9] compiles it in CI and every cycle is
compared with the proved reference. The gateway firmware reuses the DBC through
cantools-generated C and the reference gateway logic in C; the same CAN messages
mean the ROS 2 `can_bridge` and the remote console work unchanged.

## References

1. D. Seward, D. Bradley, R. Bracewell, "The development of research models for automatic excavation", 5th ISARC, Tokyo, 1988. [PDF](https://www.iaarc.org/publications/fulltext/The_development_of_research_models_for_automatic_excavation.PDF)
2. S. Brennan, A. Alleyne, "The Illinois Roadway Simulator: a mechatronic testbed for vehicle dynamics and control", IEEE/ASME Trans. Mechatronics, 2000. [record](https://pure.psu.edu/en/publications/the-illinois-roadway-simulator-a-mechatronic-testbed-for-vehicle-/)
3. S. Brennan, "Similarity conditions for comparing closed-loop vehicle roll and pitch dynamics", 2004. [PDF](https://www.me.psu.edu/toboldlygo/Publications/Conferences/2004_Brennan_SimilarityConditionsForComparingClosedLoopVehicleRollAndPitchDynamics.pdf)
4. M. Kloock et al., "Networked and autonomous model-scale vehicles for experiments in research and education", 2020. [arXiv:2004.08364](https://arxiv.org/pdf/2004.08364)
5. K. Aoshima, M. Servin, "Examining the simulation-to-reality gap of a wheel loader digging in deformable terrain", 2024. [arXiv:2310.05765](https://arxiv.org/pdf/2310.05765)
6. SICK, nanoScan3 Pro I/O NANS3-CAAZ30AN1 data sheet. [PDF](https://www.sick.com/media/pdf/0/80/980/dataSheet_NANS3-CAAZ30AN1_1100334_en.pdf)
7. ISO 14397-1:2007, Earth-moving machinery - Loaders and backhoe loaders - Part 1: Calculation of rated operating capacity and test method for verifying calculated tipping load. [ISO](https://www.iso.org/standard/39806.html)
8. ISO 3691-4:2023, Industrial trucks - Safety requirements and verification - Part 4: Driverless industrial trucks and their systems. [preview](https://webstore.ansi.org/preview-pages/ISO/preview_ISO+3691-4-2023.pdf)
9. matiec IEC 61131-3 compiler (Beremiz). [GitHub](https://github.com/beremiz/matiec)
10. E. Halbach, A. Halme, V. Kyrki, "Investigation of robotic material loading strategies using an earthmoving simulator", EUROSIM 2016 (Aalto; compact loader model). [PDF](https://acris.aalto.fi/ws/portalfiles/portal/31662253/ecp17142162.pdf)
11. Progressive Automations PA-14P data sheet. [PDF](https://cdn.shopify.com/s/files/1/0061/7735/7891/files/Linear_Actuator_with_Potentiometer_PA-14P.pdf?11196)
12. CODESYS, CAA CAN Low Level library reference. [docs](https://content.helpme-codesys.com/en/libs/CAA%20Can%20Low%20Level%20Extern/3.5.16.0/CAN-Low-Level/Functions/Basic-Functions/fld-Basic-Functions.html)
