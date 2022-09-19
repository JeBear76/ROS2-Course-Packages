#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import HardwareStatus


class HardwareStatusPublisher(Node):
    def __init__(self, nodeName):
        self.nodeName = nodeName
        super().__init__(nodeName)
        self._publisher = self.create_publisher(
            HardwareStatus, 'hardware_status', 10)
        self.create_timer(2, self.publishHardwareStatus)
        self.get_logger().info(f"Node {nodeName} started.")

    def publishHardwareStatus(self):
        msg = HardwareStatus()
        msg.temperature = 274
        msg.are_motors_ready = False
        msg.debug_message = 'You have burnt the motors'
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisher("hardware_status_publisher")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
