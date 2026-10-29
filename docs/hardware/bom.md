# Retrofit kit: bill of materials (candidate parts)

Target machine: articulated wheel loader, 14–18 t, 3 m³ bucket, CAN-controlled
transmission and electro-hydraulic pilot controls (most loaders built after
about 2015). Older loaders with hydraulic pilot joysticks need the
electro-hydraulic pilot kit in section E.

**Every part number below is a candidate to confirm with the supplier.** None
has been bought or bench-tested. The table records why each part was picked,
what it has to achieve, and which ones carry safety functions. Prices are
left out on purpose: get quotes from the suppliers before costing the kit.

Safety-relevant parts (S) belong to the independent safety channel and are
chosen for certified performance (ISO 13849-1 PL d / IEC 61508 SIL 2, or
IEC 62998 for outdoor sensors). Non-safety parts (N) serve productivity and
autonomy.

## A. Safety channel (S)

| # | Function | Candidate | Qty | Requirement it meets | Notes / to confirm |
|---|---|---|---|---|---|
| A1 | Safety controller | TTControl HY-TTC 500 family (safety variant) or ifm ecomatController safety | 1 | PL d, 2 CAN, ≥ 8 safe digital in, ≥ 4 safe outputs, 25 ms task | Runs `kauha/safety/plc.py` logic, ported to the vendor's certified toolchain (C / CODESYS Safety). |
| A2 | Safety laser scanner, front | SICK outdoorScan3 | 2 | Outdoor certified (IEC TS 62998), 275° scan angle, 4 m protective range, field-set switching by inputs, up to 128 fields with 8 monitored at once | The 4 m protective range caps the autonomous speed (3.17 m/s dry, 2.61 m/s winter; see `stopping.py`). Heated window option for winter. |
| A3 | Safety laser scanner, rear | SICK outdoorScan3 | 2 | as A2 | Mount at the rear corners turned 45° outward, so each one also covers its side of the rear frame. Keep clear of the rear grille. |
| A4 | Driveline speed sensor | Inductive, two-channel, on the transmission output | 1 | Independent speed for SR-04/SR-08 | Must not share a signal path with the OEM speed on CAN. |
| A5 | Articulation angle sensor | Contactless angle sensor, safety rated | 1 | Field-set sector selection (SR-02) | Mount on the articulation joint pin. |
| A6 | Safety GNSS receiver | Standard GNSS + SBAS, separate antenna | 1 | Geofence (SR-07), with dead reckoning from A4/A5 | Not RTK on purpose: independent of the autonomy GNSS. |
| A7 | Remote e-stop radio | HBC radiomatic or Autec safety radio | 1 | PL d e-stop over radio, loss-of-signal = stop | Separate from the data radio (C3). |
| A8 | Machine e-stop buttons | Mushroom head, two-channel, cab + both sides | 3 | SR-01 | Wired to A1 safe inputs. |
| A9 | Brake / drive cut relays | Force-guided relays, two-channel | 2 | Hardwired stop: park brake apply, transmission neutral | Category 3 architecture with A1 monitoring the feedback contacts. |
| A10 | Hydraulic pilot cut-off valve | 24 V normally-closed cartridge valve on the pilot supply | 1 | Implement stop independent of the gateway | Fail-safe: de-energised = no pilot pressure. |
| A11 | Beacon + horn | Amber beacon, reversing alarm | 2 | Warns people of autonomous mode | Driven by A1. |

## B. Perception and positioning (N)

| # | Function | Candidate | Qty | Requirement it meets | Notes / to confirm |
|---|---|---|---|---|---|
| B1 | 3D lidar | Ouster OS1-64/128 | 1 | Pile face, truck pose, landmarks, people at 0.5–60 m | IP68/IP69K. Cab roof mount, cleaning nozzle. |
| B2 | 4D imaging radar | Continental ARS548 RDI | 1 | Sees through dust, snow and fog | Fallback for the lidar in dust; not used in the twin yet. |
| B3 | Cameras | Automotive GMSL2 cameras, heated housings | 4 | Person/PPE detection (front, rear, two sides), remote video | Detector trained in `train_ppe.py`. |
| B4 | RTK GNSS + heading | Septentrio mosaic-H (dual antenna) or equivalent | 1 | 2 cm position, heading for the EKF | Needs an RTK correction source (site base station or network). |
| B5 | IMU | Industrial MEMS IMU (or the IMU in B4) | 1 | Gyro for EKF prediction | |
| B6 | Retroreflective posts | 100 mm reflective tape on 2 m posts | 8–12 | Localisation through GNSS outages | Positions surveyed once; listed in the site config. |

## C. Compute and communications (N)

| # | Function | Candidate | Qty | Requirement it meets | Notes / to confirm |
|---|---|---|---|---|---|
| C1 | Autonomy computer | NVIDIA Jetson AGX Orin, industrial carrier, −25…+70 °C | 1 | Runs `kauha/autonomy`, detector inference | Shock and vibration rating for the cab mounting. |
| C2 | CAN interface | 2–4 channel CAN-FD USB/PCIe interface (socketcan) | 1 | Machine bus J1939 to the gateway | |
| C3 | Data radio | Private LTE/5G router with two modems | 1 | Teleoperation video and commands; target p95 < 150 ms | Latency model in `kauha/sim/remote.py`. |
| C4 | Ethernet switch | Managed industrial switch, M12 | 1 | Lidar, radar, cameras, compute | |
| C5 | Event recorder storage | Industrial SSD, 1 TB | 1 | Hash-chained safety log + data logging | |

## D. Drive-by-wire gateway (N, but monitored)

| # | Function | Candidate | Qty | Notes / to confirm |
|---|---|---|---|---|
| D1 | Gateway ECU | Mobile controller with 3 CAN | 1 | Bridges the KAUHA bus to the OEM bus. Applies the PLC speed limit and command timeouts (`kauha/can/gateway.py`). Its OEM-side messages are machine specific and need the OEM's interface documentation or approval. |
| D2 | Harness | Deutsch DT connectors, shielded twisted pair for CAN | 1 set | |

## E. Option for machines without electro-hydraulic pilots

| # | Function | Candidate | Qty | Notes |
|---|---|---|---|---|
| E1 | Electro-hydraulic pilot valve block | Proportional pilot valves (e.g. Danfoss PVG with PVE actuation) | 1 | Replaces the joystick pilot circuit for boom, bucket and steering. |
| E2 | Steering valve | Electro-hydraulic steering priority valve | 1 | |
| E3 | Mode selector | Key switch: MANUAL / AUTO / TELEOP | 1 | A1 input. Manual mode removes power from the gateway outputs. |

## What the kit costs in time, not money

- Installation: 3–5 days per machine for a trained technician (estimate,
  confirm in the pilot).
- Site setup: survey the posts and zones, about 1 day.
- Commissioning: field-set verification with a test body at each speed
  band, brake tests in dry and winter conditions, about 2 days.
