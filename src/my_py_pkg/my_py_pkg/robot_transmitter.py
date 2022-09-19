#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotTransmitterNode(Node):
    def __init__(self, nodeName):
        self.nodeName = nodeName
        super().__init__(nodeName)

        self.declare_parameter("station_name", "Robot_FM")
        self._station_name = self.get_parameter("station_name").value

        self.pub = self.create_publisher(String, self._station_name, 10)
        self.create_timer(1.0, self.radio_FM_transmit)

        self.get_logger().info(
            f"Node {nodeName} started with Station name {self._station_name}.")

    def radio_FM_transmit(self):
        msg = String()
        msg.data = f"Breaking News! Python {self._station_name} is up!"
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTransmitterNode("robot_transmitter")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
