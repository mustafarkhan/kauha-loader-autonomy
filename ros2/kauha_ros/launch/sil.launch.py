"""Multi-process SIL with a ROS 2 autonomy computer.

    ros2 launch kauha_ros sil.launch.py speed:=2.0 person:='{"t": 8, "ahead": 8}'

machine, safety PLC and remote station run as separate processes on a UDP-multicast CAN
bus (the SIL of kauha/sil). The autonomy computer is ROS 2: straight_driver publishes
/cmd_drive, can_bridge puts it on the CAN bus and publishes the safety PLC's status.
"""
import json
import random
import sys
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    group = f"239.74.{random.randint(1, 254)}.{random.randint(1, 254)}"
    port = random.randint(20000, 40000)
    epoch = time.monotonic() + 4.0              # ROS nodes and processes start their loops together
    duration = lc("duration")
    common = ["--group", group, "--port", str(port), "--epoch", f"{epoch:.6f}", "--duration", duration]
    procs = [ExecuteProcess(cmd=[sys.executable, "-m", "kauha.sil.proc", role, *common, *extra], output="screen")
             for role, extra in (("machine", ["--person", lc("person")]), ("plc", []), ("remote", []))]
    return procs + [
        Node(package="kauha_ros", executable="can_bridge", output="screen",
             parameters=[{"interface": "udp_multicast", "channel": group, "mode": 1}]),
        Node(package="kauha_ros", executable="straight_driver", output="screen",
             parameters=[{"speed": float(lc("speed")), "start_delay": 5.5}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("speed", default_value="2.0"),
        DeclareLaunchArgument("duration", default_value="20.0"),
        DeclareLaunchArgument("person", default_value=json.dumps({"t": 12, "ahead": 8})),
        OpaqueFunction(function=_setup),
    ])
