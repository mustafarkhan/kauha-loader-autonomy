# Hardware design package

| Document | Contents |
|---|---|
| [bom.md](bom.md) | Candidate parts, grouped into the safety channel, perception, compute, gateway and retrofit options |
| [interface_spec.md](interface_spec.md) | CAN/J1939 messages, rates, timeouts, end-to-end protection, hardwired safety I/O, stop timing budget |
| [wiring.md](wiring.md) | System architecture and wiring principles, with notes for winter harnesses |
| [sensor_mounting.md](sensor_mounting.md) | Sensor positions and fields of view, and the limits to check on the machine |
| [hil_bench.md](hil_bench.md) | Hardware-in-the-loop bench: layout, equipment, test cases mirrored from CI, exit criteria |
| [`../../hardware/can/kauha_machine.dbc`](../../hardware/can/kauha_machine.dbc) | Machine-bus DBC |
| [`../../hardware/config/loader_medium.yaml`](../../hardware/config/loader_medium.yaml) | Machine parameters used by the twin and the autonomy |
| [`../../hardware/config/site_demo.yaml`](../../hardware/config/site_demo.yaml) | Demo site: piles, zones, truck bay, survey posts |

The machine parameters are representative of the 15 t class. They are not
taken from a specific manufacturer's data sheet. Replace them with the target
machine's data, and with measured values for braking and actuator latency,
before using any result quantitatively.
