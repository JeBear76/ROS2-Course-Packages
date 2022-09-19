from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtle_sim_display = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_spawn_node = Node(
        package="assignment_turtle_sim_project",
        executable="turtle_spawn",
        parameters=[{"spawn_frequency": 5}, {"spawn_immediately": False}]
    )

    turtle_control_node = Node(
        package="assignment_turtle_sim_project",
        executable="turtle_control",
        parameters=[{"turtle_speed": 1.5}]
    )

    ld.add_action(turtle_sim_display)
    ld.add_action(turtle_spawn_node)
    ld.add_action(turtle_control_node)

    return ld
