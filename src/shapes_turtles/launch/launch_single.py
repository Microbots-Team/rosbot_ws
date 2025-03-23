from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                name="sim",
            ),
            Node(
                package="shapes_turtles",
                executable="draw_shape",
                name="artist",
            ),
        ]
    )
