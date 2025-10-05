from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Hello World Node
            Node(
                package="wheatley",
                executable="hello_world_node",
                name="hello_world_node",
                output="screen",
            ),
        ]
    )
