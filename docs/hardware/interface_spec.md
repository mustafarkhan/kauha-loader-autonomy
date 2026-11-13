# Interface specification

## 1. Networks

| Network | Physical | Participants | Purpose |
|---|---|---|---|
| KAUHA machine bus, safety segment | CAN 2.0B, 500 kbit/s, J1939 29-bit IDs, inside the control cabinet and machine harness only | Safety PLC (SA 0x81), drive-by-wire gateway (0x83), sensor node (0x84), field evaluation (0x85) | Safety status, machine state, the PLC's sensor frames |
| KAUHA machine bus, open segment | CAN 2.0B, 500 kbit/s, J1939 29-bit IDs | Autonomy computer (0x80), remote radio gateway (0x82), zone source (0x86), the gateway's second port | Commands with authentication frames (PGN 0xFF60), challenge (0xFF61), a copy of the safety segment's traffic. The gateway's segment guard forwards only authenticated commands to the safety segment (`kauha/can/guard.py`) |
| OEM machine bus | CAN (J1939), machine specific | Drive-by-wire gateway, OEM ECUs | Transmission, steering, hydraulics; reached only through the gateway |
| Sensor Ethernet | 1000BASE-T1 / M12 | Lidar, radar, cameras, autonomy computer | Perception data; non-safety |
| Hardwired safety I/O | 24 V two-channel | Safety PLC, scanners, e-stops, relays, pilot cut-off valve | Stop functions; independent of both CAN buses |

The DBC is `hardware/can/kauha_machine.dbc`, the encoder/decoder is
`kauha/can/codec.py`, and the CAN tests are in `tests/can` and `tests/hil`.

## 2. Messages

| Message | J1939 ID | PGN | Sender | Rate | Receiver timeout | E2E |
|---|---|---|---|---|---|---|
| DriveCmd | 0x0CFF1080 | 0xFF10 | Autonomy | 20 Hz | 150 ms gateway, 200 ms PLC watchdog | counter + CRC8 |
| ImplementCmd | 0x0CFF1180 | 0xFF11 | Autonomy | 20 Hz | 150 ms | counter + CRC8 |
| SafetyStatus | 0x08FF2081 | 0xFF20 | Safety PLC | 40 Hz | 100 ms | counter + CRC8 |
| MachineState1 | 0x0CFF3083 | 0xFF30 | Gateway | 40 Hz | — | counter + CRC8 |
| MachineState2 | 0x0CFF3183 | 0xFF31 | Gateway | 20 Hz | — | counter + CRC8 |
| RemoteHeartbeat | 0x0CFF4082 | 0xFF40 | Remote GW | 20 Hz | 500 ms PLC | counter + CRC8 |

### End-to-end protection

- Byte 6 low nibble: alive counter. It increments by 1 per frame and wraps at 16.
- Byte 7: CRC-8, SAE J1850 (polynomial 0x1D, init 0xFF, final XOR 0xFF).
  It is computed over the data ID followed by bytes 0–6.
  - The data ID is the low byte of the PGN, so a frame that appears on the
    wrong identifier fails the check.
  - Check value: CRC("123456789") = 0x4B.
- The receiver rejects the following:
  - CRC errors;
  - a repeated counter;
  - a counter jump larger than 2;
  - a frame older than the timeout. In that case the gateway applies the
    brake and holds the implement.
- This follows the pattern of the AUTOSAR E2E profiles. It is not a certified
  implementation, so the certified safety PLC toolchain must supply its own
  version.

### Signals

The DBC is the reference. Key points:

- **DriveCmd**
  - SpeedReq: signed, 0.001 m/s per bit.
  - ArticulationReq: signed, 0.0001 rad per bit.
  - Mode: 0 IDLE, 1 AUTO, 2 TELEOP.
  - BrakeReq.
  - DigFieldReq / DockFieldReq: requests to the PLC for the special field sets.
- **SafetyStatus**
  - State: 0 RUN, 1 WARNING, 2 PROTECTIVE_STOP, 3 EMERGENCY_STOP.
  - ResetPending.
  - ActiveFieldSet.
  - Cause: bitfield, see `kauha.safety.plc.Cause`.
  - DockBlocked: the truck driver's cab door is open (SR-14); the autonomy holds short of the truck.
  - SpeedLimit: 0.001 m/s per bit.
  - This message is informational. The stop itself is hardwired.
- **RemoteHeartbeat**
  - EStop.
  - ResetReq: the PLC acts on the rising edge.
  - ModeReq.

## 3. Hardwired safety I/O (safety PLC)

| Signal | Direction | Channels | Function | Requirement |
|---|---|---|---|---|
| Front/rear scanner OSSD pairs (protective) | in | 2 × 2 per scanner | Protective field violated | SR-02 |
| Scanner warning outputs | in | 1 per scanner | Warning field violated | SR-03 |
| Scanner contact-band outputs (second simultaneous field) | in | 2 per scanner | Someone or something within 0.35 m of the frame side; blocks teleop side recovery | SR-12 |
| Front scanner under-bucket outputs, near, mid and far | in | 2 + 2 + 2 per front scanner | Ground between the front tyres and the bucket (near: 2.0–2.8 m; mid: 2.8–3.35 m, short of the lowered bucket's back edge), and under the raised bucket (far) | SR-16 |
| Front scanner front-side output | in | 2 per front scanner | Beside the front frame, evaluated in every set (used while reversing) | SR-16 |
| Boom angle | in | 2 (two independent sensors) | Bucket bottom above 1.0 m: the scan plane passes under the bucket | SR-16 |
| Field-set select | out | 4–6 per scanner (encoded) | Selects speed/direction/sector set | SR-02, SR-10 |
| Machine e-stops (3) | in | 2 each | Emergency stop | SR-01 |
| Safety radio e-stop | in | 2 | Remote e-stop, loss of signal = stop | SR-01 |
| Driveline speed sensor | in | 2 | Independent speed | SR-04, SR-08 |
| Articulation angle | in | analog/encoder | Field sector | SR-02 |
| Mode key switch | in | 2 | MANUAL / AUTO / TELEOP | — |
| Truck cab-door switch (via the bay's safety radio receiver) | in | 2 | Driver out of the cab: no dock field set, stop if docking | SR-14 |
| Park brake + transmission neutral relays | out | 2 (force-guided, monitored) | Drive stop | all stops |
| Pilot supply cut-off valve | out | 1 + feedback | Implement stop | all stops |
| Beacon, horn | out | 1 each | Warning to people | — |

Stop category: protective stops are category 1 (brake under control, then
remove power) via the service brake and transmission neutral. The park
brake is applied once the machine is at standstill or after 2 s.

## 4. Timing budget for a protective stop

| Stage | Budget |
|---|---|
| Scanner response (incl. multiple sampling) | 100 ms (confirm for the selected scanner configuration) |
| Safety PLC (2 cycles × 25 ms) | 50 ms |
| Relay + brake valve actuator latency | 120 ms |
| Brake pressure build-up | 200 ms |
| **Total reaction before full deceleration** | **470 ms** |

Stopping distance = v·0.47 s + v²/(2a) + 0.5 m.

- a = 2.5 m/s² dry and 1.5 m/s² winter. Both are to be measured on each
  machine type during commissioning.
- The 4 m protective range gives 3.17 m/s dry and 2.61 m/s winter as the
  highest protected speeds.
