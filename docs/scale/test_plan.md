# 1:8 testbed: commissioning and test plan

Each test has a setup, steps and a pass criterion. Record results in
`docs/scale/test_records.csv` (one row per run: date, test, step, measured value,
pass/fail, who). A failed step stops the sequence until it is understood.

## Rules for every test

1. **No person in the test area while the machine is energised.** The test area is
   a fenced or taped zone at least 3 m beyond the machine's reach. Test pieces
   stand in for people (step T4). In the evaluator profile nothing on the machine
   is rated to protect a person.
2. The test conductor holds the radio e-stop at all times and stays outside the area.
3. A second e-stop (SB3) is on the bench, within reach of the laptop.
4. Before each session: walk-around, battery voltage, e-stop test (T2.1).
5. Speeds on the floor start at the lowest band and step up only after the band below
   has passed T3.

## T0: bench, power off

| Step | Check | Pass |
|---|---|---|
| T0.1 | Continuity of both e-stop channels with all e-stops released; open with each pressed | every e-stop opens both channels |
| T0.2 | K2..K5 NC contacts: motor phases short-circuited through the resistors with coils off | three phases joined, about 0.94 ohm phase-to-phase |
| T0.3 | No connection between 36 V traction and 24 V safety circuits | > 1 Mohm |
| T0.4 | CAN: 60 ohm between CAN_H and CAN_L with both terminators, on each of the two segments; no continuity between the segments | 55-65 ohm each; > 1 Mohm between them |

## T1: bench, wheels off the ground, power on

| Step | Check | Pass |
|---|---|---|
| T1.1 | `candump can0`: all nodes transmit at their rates (PLC 40 Hz, gateway 40/20 Hz, sensor node 40 Hz, evaluator 40 Hz, remote 20 Hz, zones 10 Hz) | rates within 10 % |
| T1.2 | PLC boots into EMERGENCY_STOP; after heartbeat + reset (remote ResetReq and SB4) it reports RUN | as stated |
| T1.3 | Potentiometer calibration: drive each actuator to both ends at low duty, record counts into `pot_retracted` / `pot_extended` | lengths from the tables within 2 mm of a ruler at 3 points |
| T1.4 | AS5600 zero offsets: frames straight, boom at -0.2 rad (carry, set with the CAD template); store `art_zero`, `boom_zero` | sensor node articulation within 0.01 rad of the gateway's |
| T1.5 | Hall count: turn the FL wheel by hand 10 revolutions | 900 +/- 2 steps; else set `HALL_STEPS_PER_REV` |
| T1.6 | ODrive: calibrate each axis (motor + Hall encoder), set pole pairs 15, velocity limit 2.4 turns/s | `Get_Encoder_Estimates` velocity matches a tachometer within 3 % |

## T2: stop functions on the bench (wheels off the ground)

Run each with the wheels spinning at 0.3 m/s equivalent, in TELEOP. Measure with a
logic analyser or `candump -t a` plus a scope on the relay coils.

| Step | Stimulus | Pass |
|---|---|---|
| T2.1 | Press each e-stop (SB1, SB2, SB3, radio) | K2..K6 drop within 30 ms; wheels stop; PLC state 3 with ESTOP_MACHINE or ESTOP_REMOTE |
| T2.2 | Unplug the remote / stop RemoteHeartbeat | stop between 0.5 and 0.6 s, cause REMOTE_LINK |
| T2.3 | Kill the autonomy computer's DriveCmd | stop between 0.2 and 0.3 s, cause WATCHDOG |
| T2.4 | Disconnect the sensor node | stop within 0.15 s, cause SCANNER_FAULT |
| T2.5 | Corrupt frames (inject a frame with a wrong CRC every cycle with `cansend`) | frames ignored; no state change |
| T2.6 | Evaluator echoes a wrong field set (test mode flag) | SCANNER_FAULT within 0.15 s |
| T2.7 | Hold K2 mechanically closed (test fixture) and command a stop | relay fault latched; restart refused until power cycle |
| T2.8 | Stop the Pi (pull power) | KP1/KP2 drop; wheels stop |
| T2.9 | Reset with a cause still present | refused (reset_pending false) |
| T2.10 | Segment guard: from the open segment send (a) a SafetyStatus frame with a correct CRC, (b) a DriveCmd with no companion frame, (c) a recorded DriveCmd and companion pair again, (d) the same pair after power-cycling the gateway | none appears on the safety segment (`candump can0` on the PLC); the machine stops by WATCHDOG while only forged commands arrive |
| T2.11 | Gateway without keys (erased EEPROM) | no command is forwarded, LED blinks fast, PLC reports WATCHDOG |

## T3: stopping distance on the floor (no people, test pieces only)

Purpose: replace the planning values in the stop budget with measured ones. Mark a
start line, drive through it in TELEOP at the band speed, trigger the stop with the
stimulus at the line (e-stop, field input forced by the test fixture), measure the
distance from the line to the stopped bucket edge with a tape or the overhead camera.

| Band | Speed (m/s) | Runs | Pass |
|---|---|---|---|
| 1 | 0.18 | 10 | max distance <= protective field of the band (generated table) less 62.5 mm margin |
| 2 | 0.53 | 10 | same |
| 3 | 0.88 | 10 | same |
| 4 | cap (0.82 evaluator / 1.00 nanoScan3) | 10 | same |

Then compute the deceleration and reaction time from the runs (fit d = v t_r + v^2 / 2a
to the 40 runs), set `TESTBED_DECEL_M_S2`, `RELAY_DROPOUT_S` and the scanner response in
`kauha/scale/profile.py`, regenerate (`python -m kauha.scale.profile`,
`python firmware/generate.py`), rerun `pytest tests/plc tests/scale`, re-import into
CODESYS, reload the field sets, and repeat the band-4 runs. Repeat on a second
surface (dry concrete and loose sand or gravel); the lower deceleration governs.

## T4: field checks with test pieces

Test pieces: a 70 mm diameter cylinder 400 mm long lying down and a 200 mm diameter
cylinder 600 mm high standing, the shapes ISO 3691-4 uses for driverless trucks.
Machine at standstill, then creeping in TELEOP.

| Step | Check | Pass |
|---|---|---|
| T4.1 | Standing piece at the edge of each front/rear protective field of the standstill set and of one set per band | stop when the piece is inside, no stop 50 mm outside |
| T4.2 | Lying piece across the path in front of the bucket, machine creeping forward | stop before contact |
| T4.3 | Standing piece beside the front frame while reversing (front-side field) | stop |
| T4.4 | Standing piece in the gap between tyres and lowered bucket (SR-16 mid field) | stop; no restart while it stays (recovery refused) |
| T4.5 | Piece under the raised bucket | stop and boom hold |
| T4.6 | Dig zone: piece in the pile face area with DigFieldReq inside the zone | front field muted, side fields active; muting stops after 8.8 s |

## T5: teleoperation and recovery

| Step | Check | Pass |
|---|---|---|
| T5.1 | Console: take teleop, drive the course at creep and at band 2 | speed never above the PLC's SpeedLimit (MachineState1 vs SafetyStatus) |
| T5.2 | Field stop, then supervisor reset with the field still occupied | recovery mode, creep 0.18 m/s away only; moving toward the piece trips again |
| T5.3 | Link loss during teleop | stop within 0.6 s |

## T6: autonomous V-cycle

Pile of dry sand (bunker 1.5 x 1.0 m, 0.3 m high), a hopper box 0.5 x 0.3 m, the site
scaled from `hardware/config/site_carry.yaml`. ROS 2 stack with the scale profile.

| Step | Check | Pass |
|---|---|---|
| T6.1 | 20 cycles without intervention, nobody in the area | no unplanned stop; cycle time recorded (Froude target: twin cycle / 2.83) |
| T6.2 | Test piece placed on the route during a cycle (by a pole from outside) | stop, request to the console, resume after removal and reset |
| T6.3 | Compare logged speeds, stops and cycle phases with a twin run of the same site at 1:8 | report differences; they become the next model updates |
