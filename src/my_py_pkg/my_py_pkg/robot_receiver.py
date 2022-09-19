#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotReceiverNode(Node):
    def __init__(self, nodeName):
        self.nodeName = nodeName
        super().__init__(nodeName)

        self.declare_parameter("station_name", "Robot_FM")
        self._station_name = self.get_parameter("station_name").value

        self.receiver_ = self.create_subscription(
            String, self._station_name, self.robot_FM_callback, 10)

        self.get_logger().info(f"Node {nodeName} started.")

    def robot_FM_callback(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = RobotReceiverNode("robot_receiver")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
