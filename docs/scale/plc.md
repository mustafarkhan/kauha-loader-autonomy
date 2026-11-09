# Safety PLC program (CODESYS)

`plc/codesys/` holds the testbed's safety program in IEC 61131-3 Structured Text.

| File | What |
|---|---|
| `src/DUT_Types.st` | configuration, input and output structures, CAN payload |
| `src/FUN_Helpers.st` | FMIN/FMAX/HAS (reproduce the reference's tie rules exactly) |
| `src/FUN_Can.st` | CRC-8 J1850, E2E receiver `FB_E2ERx`, signal access, SafetyStatus encoder |
| `src/FB_SafetyCore.st` | one 25 ms cycle of the safety logic: line-by-line port of `kauha/safety/core.py` |
| `src/FB_SafetyNode.st` | frames in, checks, builds the core's inputs, outputs and the SafetyStatus frame |
| `src/GVL_Config.st` | generated: `CFG_FULL`, `CFG_SCALE8_NANOSCAN3`, `CFG_SCALE8_EVALUATOR` |
| `src/codesys_only/FB_CanPort.st` | CAN driver calls (CAA CAN Low Level); the only vendor-library code |
| `src/codesys_only/GVL_IO.st` | GPIO / I/O channels |
| `src/codesys_only/PRG_Main.st` | the cyclic program |
| `import_into_codesys.py` | creates all of the above in an open CODESYS project |

Everything in `src/` (not `codesys_only/`) is plain IEC 61131-3 and is compiled in
CI with the matiec compiler, then compared cycle by cycle with the proved Python
reference (`python -m kauha.verify.st_equivalence`, `tests/plc`).

## Setting up the testbed PLC

1. **Hardware.** Raspberry Pi 4 with a Waveshare RS485 CAN HAT (MCP2515) and the
   24 V input and relay boards of [wiring.md](wiring.md). Raspberry Pi OS (64-bit).
   Enable the HAT: add `dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25` to
   `/boot/firmware/config.txt` (use the crystal frequency printed on your HAT: 8, 12
   or 16 MHz), and bring the bus up at boot: `ip link set can0 up type can bitrate 500000`.
2. **CODESYS.** Install CODESYS V3.5 SP17 or later on a PC and the "CODESYS Control
   for Raspberry Pi SL" package, then deploy the runtime to the Pi (Tools > Update
   Raspberry Pi). Without a licence the runtime stops after two hours; buy one before
   unattended tests.
3. **Project.** New "Standard project", device "CODESYS Control for Raspberry Pi 64
   SL", language ST. Then Tools > Scripting > Execute Script File... and choose
   `plc/codesys/import_into_codesys.py`. It creates 25 objects.
4. **Libraries.** Library Manager > Add: "CAA CAN Low Level" (namespace CL2). Build.
   If the compiler reports a parameter mismatch in `FB_CanPort`, your library version
   differs from the documented one; adjust only that block.
5. **Task.** Task Configuration > MainTask: cyclic, `t#25ms`, watchdog enabled at
   50 ms. Call `PRG_Main`.
6. **CAN.** Device tree > Add device > CANbus, network 0, 500 kbit/s (it maps to
   SocketCAN `can0`).
7. **I/O.** Add the "GPIOs B+/Pi2/Pi3/Pi4" device and map each channel to the
   `GVL_IO` variable given in its comment (pins in [wiring.md](wiring.md)).
8. **Profile.** `PRG_Main` uses `CFG_SCALE8_EVALUATOR`. When the rated scanners are
   fitted, switch to `CFG_SCALE8_NANOSCAN3` and set `GVL_IO.USE_SCANNER_IO := TRUE`.

## Changing the safety logic

1. Change `kauha/safety/core.py` and prove it: `python -m kauha.verify.prove`.
2. Port the change to `FB_SafetyCore.st`.
3. `python -m kauha.verify.st_equivalence --cycles 200000` must report zero
   mismatches on all six profiles.
4. Rerun the import script in CODESYS and download.

## Changing measured values

After commissioning test T3 (stopping distance), set `TESTBED_DECEL_M_S2`,
`RELAY_DROPOUT_S` and the scanner response in `kauha/scale/profile.py`, then
`python -m kauha.scale.profile`. That rewrites `GVL_Config.st` and the field sets.
Load the new field sets into the scanners (or the evaluator), rerun the tests and
re-import into CODESYS.

## What the PLC does with missing data

| Input | When stale or corrupt | Result |
|---|---|---|
| RemoteHeartbeat | age > 0.5 s | REMOTE_LINK stop (in AUTO/TELEOP) |
| DriveCmd | age > 0.2 s | WATCHDOG stop (in AUTO/TELEOP) |
| MachineState1 (gateway speed) | > 0.1 s | speed reads 1000 m/s, SPEED_DISCREPANCY stop after 0.2 s |
| SafeSensors | > 0.1 s or SensorOk = 0 | SCANNER_FAULT stop; last speed held |
| ScannerStatus (evaluator) | > 0.1 s, ScannersOk = 0, or a set the PLC did not select in the last 4 cycles | SCANNER_FAULT stop, every field reads occupied |
| ZoneStatus | > 0.5 s | position invalid, outside all zones: POSITION_INVALID stop after 2 s in AUTO |
| E-stop chain (IN1) | open | EMERGENCY_STOP |
| Relay feedback (IN2) | coils off but mirrors open > 100 ms | latched relay fault, reads as e-stop until power cycle |
| Power-up | before any frame | EMERGENCY_STOP (remote e-stop assumed pressed) until a heartbeat and a reset |
