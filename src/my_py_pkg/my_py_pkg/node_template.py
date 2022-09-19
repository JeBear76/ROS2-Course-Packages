#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node):
    def __init__(self, nodeName):
        self.nodeName = nodeName
        super().__init__(nodeName)
        self.get_logger().info(f"Node {nodeName} started.")


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode("node_name")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
