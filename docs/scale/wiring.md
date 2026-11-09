# 1:8 testbed: wiring

Drawing: `hardware/scale/wiring/kauha_scale8_wiring.pdf` (sheet 1 power and stop
chain, sheet 2 CAN). This page is the wire list.

## Principles

1. Stop means de-energise. No stop needs a working CAN bus, Pi or gateway.
2. Every e-stop has two NC contacts, wired as two separate channels to the safety
   relay K1 (PNOZ s4): category 3 structure.
3. K1 monitors the relays it drives through their NC mirror contacts: a welded
   contact blocks the next start.
4. The PLC (not rated) repeats the decision: its relays KP1/KP2 sit in series after
   K1's safety contacts, so either can stop the machine and neither can start it alone.

## Power

| From | To | Wire | Protection |
|---|---|---|---|
| Battery 36 V + | F1 30 A, S1 main switch, power bus | 2.5 mm2 | F1 |
| Power bus | ODrive 1 and 2 DC in | 2.5 mm2 | F5 40 A |
| Power bus | U1 36->24 V 10 A, U2 36->12 V 10 A | 1.0 mm2 | F2, F3 polyfuse |
| U1 24 V | K1 A1, relay coils (via K1 / KP), scanners, 24 V input board, U3 | 0.5 mm2 | |
| U3 5 V | Pi PLC, autonomy computer (own 5 V 5 A supply if a Jetson) | 0.75 mm2 | |
| U2 12 V | K6 (K-IMPL) contacts -> 3 x MDD10A | 1.0 mm2 | F4 10 A |
| 24 V | sensor node via its own 5 V regulator (independent of U3) | 0.5 mm2 | |

## Stop chain (24 V, two colours per channel)

| Item | Connection |
|---|---|
| SB1, SB2, SB3 e-stops, KR radio e-stop relay (NC, fail-safe) | channel 1 in series to K1 S11-S12, channel 2 in series to K1 S21-S22 |
| SB4 reset (NO, blue) | K1 S33-S34 (monitored reset: K1 starts on release) |
| K2..K6 NC mirror contacts in series | K1 Y1-Y2 feedback loop, and to PLC IN2 |
| K1 13-14 | -> KP1 contact (PLC OUT1) -> coils of K2..K5 (K-DRIVE) |
| K1 23-24 | -> KP2 contact (PLC OUT2) -> coil of K6 (K-IMPL) |
| K1 33-34 | 24 V to PLC IN1 (chain closed and K1 reset) |

## K-DRIVE: one 3-pole changeover relay per wheel (Finder 60.13, 24 V)

| Contact | Connection |
|---|---|
| COM (x3) | hub motor phases A, B, C |
| NO (x3) | ODrive phase outputs |
| NC (x3) | joined through 3 x 0.47 ohm 10 W resistors (star): dynamic brake |

Coil energised: motor driven. Coil off (any stop, any broken wire): motor
disconnected from the ODrive and braking into the resistors. The ODrive sees an
open motor and faults; the gateway re-arms it (`Set_Axis_State` 8) after the next
start.

## PLC I/O (Raspberry Pi GPIO, BCM numbers)

| Signal | Pin | Board | GVL_IO variable |
|---|---|---|---|
| IN1 e-stop chain OK (K1 33-34) | GPIO5 | 24 V opto input 1 | `xEstopChainOk` |
| IN2 relay mirrors closed | GPIO6 | input 2 | `xRelayFeedbackOk` |
| IN3 front OSSD clear | GPIO12 | input 3 | `stScannerIO.front_ossd_clear` |
| IN4 rear OSSD clear | GPIO13 | input 4 | `stScannerIO.rear_ossd_clear` |
| IN5 front warning clear | GPIO19 | input 5 | `stScannerIO.front_warn_clear` |
| IN6 rear warning clear | GPIO20 | input 6 | `stScannerIO.rear_warn_clear` |
| IN7 side fields clear | GPIO21 | input 7 | `stScannerIO.side_clear` |
| IN8 under-bucket fields clear | GPIO26 | input 8 | `stScannerIO.under_clear` |
| scanners ready | GPIO4 | input board power-good | `stScannerIO.scanners_ready` |
| OUT1 KP1 drive enable | GPIO23 | relay 1 | `xDriveEnable` |
| OUT2 KP2 implement enable | GPIO24 | relay 2 | `xImplementEnable` |
| OUT3..OUT8 field-set code bits 0..5 | GPIO16, 17, 22, 27, 18, 14 | 24 V level shifter to the scanners' static inputs | `xFs0..xFs5` |
| OUT9 run lamp | GPIO15 | relay 3 | `xRunLamp` |

IN3..IN8 and OUT3..OUT8 are used only with rated scanners (`USE_SCANNER_IO`).
The CAN HAT (Waveshare RS485 CAN HAT, MCP2515) uses SPI0 (GPIO 8-11) and GPIO25 for
its interrupt; none of the pins above. GPIO14/15 are the serial console pins:
disable the console (`raspi-config` > Interface > Serial) before using them.

## Gateway (Teensy 4.1)

| Signal | Pin |
|---|---|
| CAN1 TX/RX (machine bus) | 22 / 23 -> SN65HVD230 #1 |
| CAN2 TX/RX (ODrives) | 1 / 0 -> SN65HVD230 #2 |
| Lift L PWM/DIR, pot | 2 / 3, A0 |
| Lift R PWM/DIR, pot | 4 / 5, A1 |
| Tilt PWM/DIR, pot | 6 / 7, A2 |
| Steer L PWM/DIR, pot | 8 / 9, A3 |
| Steer R PWM/DIR, pot | 10 / 11, A4 |
| K-DRIVE closed sense (opto) | 24 |

Potentiometers: 5 V supply through a 2:3 divider, read as 0..3.3 V. Calibrate
retracted/extended counts in T1 and set `pot_retracted` / `pot_extended`.

## Sensor node (Teensy 4.0)

| Signal | Pin |
|---|---|
| CAN1 TX/RX | 22 / 23 -> SN65HVD230 |
| AS5600 hinge pin (articulation) | SDA 18 / SCL 19 (Wire) |
| AS5600 boom pivot | SDA 17 / SCL 16 (Wire1) |
| Front-left hub motor Halls A/B/C via 74HC14 | 2 / 3 / 4 |

## CAN bus

500 kbit/s, 29-bit identifiers, one twisted pair with 120 ohm at the two physical
ends (PLC and remote/zone node). Stubs under 0.3 m. Message list and timing:
`hardware/can/kauha_machine.dbc` and sheet 2 of the drawing.
