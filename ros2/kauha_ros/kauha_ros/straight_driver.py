"""Minimal example autonomy node: drive straight at a set speed (for SIL and bring-up tests).

Publishes ackermann_msgs/AckermannDriveStamped on /cmd_drive at 20 Hz after a start delay.
Stop the node and the CAN bridge stops forwarding commands; the safety PLC's watchdog stops
the machine (SR-06).
"""
from __future__ import annotations

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class StraightDriver(Node):
    def __init__(self) -> None:
        super().__init__("kauha_straight_driver")
        self.declare_parameter("speed", 2.0)
        self.declare_parameter("start_delay", 2.0)
        self.pub = self.create_publisher(AckermannDriveStamped, "/cmd_drive", 10)
        self.t0 = self.get_clock().now()
        self.create_timer(0.05, self.tick)

    def tick(self) -> None:
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 < float(self.get_parameter("start_delay").value):
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(self.get_parameter("speed").value)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
