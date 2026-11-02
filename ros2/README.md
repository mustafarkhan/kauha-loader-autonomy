# kauha_ros: the ROS 2 interface

ROS 2 Jazzy package for the KAUHA autonomy computer. It follows the architecture in
the solution design. ROS 2 is the autonomy computer's middleware. The safety PLC and
the drive-by-wire gateway stay on the CAN bus, and nothing safety-related depends on
ROS.

| Piece | What it does |
|---|---|
| `twin_bridge` | Runs the full digital twin and publishes it on standard interfaces, on simulated time. Topics: `/clock`, `/joint_states`, `/tf`, `/odom`, `/localization/odom`, `/scan/<scanner>` (`LaserScan` per safety scanner), `/diagnostics` (safety PLC state), `/safety/fields`, `/site/markers`, `/people/markers`, `/kpi`. |
| `can_bridge` | The autonomy computer's CAN interface: `/cmd_drive` (`AckermannDriveStamped`, steering angle = articulation) and `/cmd_implement` go to `DriveCmd`/`ImplementCmd` frames with end-to-end protection. `MachineState` and `SafetyStatus` come back as ROS topics. It stops forwarding a command that is older than 0.1 s, so the PLC watchdog sees a dead planner. |
| `straight_driver` | A minimal example autonomy node, for bring-up and SIL tests. |
| `urdf/kauha_loader.urdf` | The articulated loader: front and rear frames, articulation joint, boom, bucket, and the four safety-scanner frames. A test checks the scanner frames against the twin's ray-caster. |
| `shift.launch.py` | Twin bridge, `robot_state_publisher` and an optional rosbag2 recorder (MCAP). |
| `sil.launch.py` | The multi-process SIL: machine, safety PLC and remote station as separate processes on a UDP-multicast CAN bus, driven by the ROS 2 nodes. |

## Run

```bash
micromamba create -f ros2/environment.yml && micromamba activate kauha-ros
pip install --no-deps -e .              # the kauha library, from the repo root
cd ros2 && colcon build && source install/setup.bash

ros2 launch kauha_ros shift.launch.py minutes:=10 real_time_factor:=4 record:=true bag:=shift_10min
ros2 launch kauha_ros sil.launch.py speed:=2.0 person:='{"t": 10, "ahead": 8}'
```

Open the MCAP bag in Foxglove, or play it with `ros2 bag play`. A recorded minute is
about 10 MB.

## Tests

```bash
cd ros2/kauha_ros
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q test/
```

- `test_geometry.py`: every scanner frame in the URDF, through the articulation joint,
  matches the twin's scanner pose for random poses and articulation angles.
- `test_nodes.py`:
  - the twin bridge publishes the shift: clock, scans with 276 beams over 275°, safety
    diagnostics and field polygons;
  - in the SIL launched as separate processes, ROS commands drive the machine over
    CAN, and the safety PLC stops it for a person ahead;
  - when the ROS driver is killed, the bridge withholds commands and the PLC watchdog
    stops the machine.

`PYTEST_DISABLE_PLUGIN_AUTOLOAD` works around the `launch_testing` pytest plugin, which
does not load with current pytest. The tests do not need it.
