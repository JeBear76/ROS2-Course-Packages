from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    led_panel_node = Node(
        package="my_cpp_pkg",
        executable="led_panel"
    )

    battery_node_0 = Node(
        package="my_cpp_pkg",
        executable="battery",
        name="battery_0",
        remappings=[("battery_level", "battery_0_level")],
        parameters=[{"battery_low_led_position": 0}]
    )

    battery_node_1 = Node(
        package="my_cpp_pkg",
        executable="battery",
        name="battery_1",
        remappings=[("battery_level", "battery_1_level")],
        parameters=[{"battery_low_led_position": 1}]
    )

    battery_node_2 = Node(
        package="my_cpp_pkg",
        executable="battery",
        name="battery_2",
        remappings=[("battery_level", "battery_2_level")],
        parameters=[{"battery_low_led_position": 2}]
    )

    battery_life_simulator_node_0 = Node(
        package="my_py_pkg",
        executable="battery_life_simulator",
        name="battery_life_simulator_0",
        remappings=[("battery_level", "battery_0_level")],
        parameters=[{"initial_battery_level": 70}]
    )

    battery_life_simulator_node_1 = Node(
        package="my_py_pkg",
        executable="battery_life_simulator",
        name="battery_life_simulator_1",
        remappings=[("battery_level", "battery_1_level")],
        parameters=[{"initial_battery_level": 30}]
    )

    battery_life_simulator_node_2 = Node(
        package="my_py_pkg",
        executable="battery_life_simulator",
        name="battery_life_simulator_2",
        remappings=[("battery_level", "battery_2_level")],
        parameters=[{"initial_battery_level": 90}]
    )

    ld.add_action(led_panel_node)
    ld.add_action(battery_node_0)
    ld.add_action(battery_node_1)
    ld.add_action(battery_node_2)
    ld.add_action(battery_life_simulator_node_0)
    ld.add_action(battery_life_simulator_node_1)
    ld.add_action(battery_life_simulator_node_2)

    return ld
