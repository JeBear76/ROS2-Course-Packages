#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import BatteryStatus


class BatteryLifeSimulatorNode(Node):
    def __init__(self, nodeName):
        self.nodeName = nodeName
        super().__init__(nodeName)

        self.declare_parameter("initial_battery_level", 100)
        self._battery_level = self.get_parameter(
            "initial_battery_level").value

        self._battery_charging = False

        self.declare_parameter("rate_of_change", 2)
        rate_of_change = self.get_parameter("rate_of_change").value

        self.pub = self.create_publisher(BatteryStatus, "battery_level", 10)
        self.create_timer(1.0/rate_of_change, self.setBatteryLevel)

        self.get_logger().info(
            f"Node {nodeName} started.")

    def setBatteryLevel(self):
        if (self._battery_charging):
            self._battery_level = self._battery_level + 10
            if (self._battery_level >= 100):
                self._battery_charging = False
        else:
            self._battery_level = self._battery_level - 10
            if (self._battery_level <= 0):
                self._battery_charging = True

        msg = BatteryStatus()
        msg.battery_level = self._battery_level
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryLifeSimulatorNode("battery_life_simulator")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
