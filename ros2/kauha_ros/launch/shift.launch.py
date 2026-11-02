"""Run a simulated KAUHA shift and publish it on ROS 2.

    ros2 launch kauha_ros shift.launch.py minutes:=10 real_time_factor:=4 record:=true

Starts the twin bridge (which owns /clock) and robot_state_publisher with the loader URDF.
With record:=true, rosbag2 records every topic to an MCAP bag for Foxglove or ros2 bag play.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("kauha_ros"))
    urdf = (share / "urdf" / "kauha_loader.urdf").read_text()
    args = [DeclareLaunchArgument(n, default_value=v) for n, v in (
        ("controller", "learned"), ("seed", "1"), ("minutes", "10.0"), ("real_time_factor", "1.0"),
        ("record", "false"), ("bag", "kauha_shift"))]
    lc = LaunchConfiguration
    twin = Node(package="kauha_ros", executable="twin_bridge", name="kauha_twin", output="screen",
                parameters=[{"controller": lc("controller"), "seed": ParameterValue(lc("seed"), value_type=int),
                             "minutes": ParameterValue(lc("minutes"), value_type=float),
                             "real_time_factor": ParameterValue(lc("real_time_factor"), value_type=float),
                             "start_delay": 3.0}])
    return LaunchDescription(args + [
        twin,
        # the shift ends when the twin exits: stop the recorder and robot_state_publisher too
        RegisterEventHandler(OnProcessExit(target_action=twin, on_exit=[EmitEvent(event=Shutdown())])),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"robot_description": urdf, "use_sim_time": True}]),
        ExecuteProcess(cmd=["ros2", "bag", "record", "-a", "-s", "mcap", "-o", lc("bag")],
                       output="log", condition=IfCondition(lc("record"))),
    ])
