from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    new_station_name = "piggy_world"
    new_topic_name = "boob"

    radio_transmitter = Node(
        package="my_py_pkg",
        executable="robot_transmitter",
        parameters=[{"station_name": new_station_name}],
        remappings=[(new_station_name, new_topic_name)],
        name=new_station_name
    )
    radio_receiver = Node(
        package="my_py_pkg",
        executable="robot_receiver",
        parameters=[{"station_name": new_station_name}],
        remappings=[(new_station_name, new_topic_name)],
        name="piggy_lover"
    )
    ld.add_action(radio_transmitter)
    ld.add_action(radio_receiver)
    return ld
