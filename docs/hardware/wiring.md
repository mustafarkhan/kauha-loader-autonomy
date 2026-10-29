# System wiring and architecture

```mermaid
flowchart LR
  subgraph SAFETY["Independent safety channel (PL d target)"]
    FS1["Front scanners x2<br/>outdoorScan3"] -- "OSSD pairs" --> PLC
    RS1["Rear scanners x2"] -- "OSSD pairs" --> PLC
    ES["E-stops x3<br/>(2-channel)"] --> PLC
    RADIO["Safety radio<br/>remote e-stop"] --> PLC
    SPD["Driveline speed<br/>sensor (2-ch)"] --> PLC
    ART["Articulation<br/>angle sensor"] --> PLC
    SGNSS["Safety GNSS"] --> PLC
    PLC["Safety PLC<br/>25 ms cycle"] -- "field-set select" --> FS1
    PLC -- "field-set select" --> RS1
    PLC -- "force-guided relays" --> BRK["Park brake +<br/>transmission neutral"]
    PLC -- "24 V" --> PILOT["Pilot supply<br/>cut-off valve"]
    PLC --> BEACON["Beacon / horn"]
  end

  subgraph AUTO["Autonomy (non-safety)"]
    LIDAR["3D lidar"] -- Ethernet --> AC
    RADAR["4D radar"] -- Ethernet --> AC
    CAMS["Cameras x4"] -- GMSL2 --> AC
    GNSS["RTK GNSS<br/>dual antenna"] --> AC
    IMU["IMU"] --> AC
    AC["Autonomy computer<br/>Jetson AGX Orin"]
    AC <-- "LTE/5G" --> REMOTE["Remote station"]
  end

  AC -- "DriveCmd / ImplementCmd<br/>(E2E)" --> BUS(("KAUHA CAN<br/>J1939 500k"))
  PLC -- "SafetyStatus (E2E)" --> BUS
  REMOTE -. "RemoteHeartbeat<br/>via radio GW" .-> BUS
  BUS --> GW["Drive-by-wire<br/>gateway"]
  BUS --> PLC
  GW -- "MachineState" --> BUS
  GW <--> OEM(("OEM machine<br/>CAN"))
  OEM --> TX["Transmission"]
  OEM --> HYD["Steering + implement<br/>hydraulics"]
  PILOT --> HYD
  BRK --> TX
```

## Principles the wiring enforces

1. **Stops never depend on software outside the safety PLC.**
   - The brake/neutral relays and the pilot cut-off valve are driven straight
     from PLC safe outputs.
   - The gateway and the autonomy computer can be dead, hung, or wrong, and
     the stop still happens.
2. **Fail-safe by de-energising.**
   - Relays and the pilot valve need power to *allow* motion.
   - A broken wire, lost power, or a failed PLC output stops the machine.
3. **Two channels where a single fault must not defeat the function.** This
   covers e-stops, OSSDs, the speed sensor and the brake relays (with
   feedback contacts read back by the PLC).
4. **The autonomy computer only *requests* special field sets** (dig, dock).
   - The PLC grants them from its own zone and speed checks.
   - A wrong request can at worst cause a nuisance stop, never a muted field
     in the wrong place.
5. **The OEM bus is reached only through the gateway.**
   - The gateway applies the PLC's speed limit and command timeouts.
   - It filters which OEM messages can be sent, which keeps cybersecurity
     exposure small (EU Machinery Regulation 2023/1230, Annex III 1.1.9).

## Harness notes for Nordic winter operation

- Use connectors rated IP67/IP69K and cables rated for flexing at −40 °C.
- Put drip loops at every sensor.
- Scanner and camera windows need heaters (24 V, about 20 W each) switched
  by a thermostat. Budget the alternator load accordingly.
- Route the CAN stubs under 0.3 m and fit 120 Ω termination at both bus ends.
  The HIL bench checks the bus with the real harness.
