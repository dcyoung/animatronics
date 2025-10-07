from pathlib import Path
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "wheatley"
PACKAGE_SHARE_DIR = Path(get_package_share_directory(PACKAGE_NAME))


def generate_launch_description():
    return LaunchDescription(
        [
            # Robot State Publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                str(PACKAGE_SHARE_DIR / "urdf" / "bot.xacro"),
                            ]
                        )
                    }
                ],
            ),
            # Joint State Publisher GUI (for manual control)
            # Node(
            #     package="joint_state_publisher_gui",
            #     executable="joint_state_publisher_gui",
            #     name="joint_state_publisher_gui",
            #     output="screen",
            # ),
            # Bot State Publisher (animates joint states)
            Node(
                package=PACKAGE_NAME,
                executable="bot_state_publisher_node",
                name="bot_state_publisher_node",
                output="screen",
            ),
            # RViz2 for visualization
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(PACKAGE_SHARE_DIR / "config" / "robot_view.rviz")],
            ),
            # # Foxglove bridge node
            # Node(
            #     package="foxglove_bridge",
            #     executable="foxglove_bridge",
            #     name="foxglove_bridge",
            #     output="screen",
            #     parameters=[{"port": 8765}],  # equivalent to port:=8765
            # ),
            # # rosbridge_server
            # Node(
            #     package="rosbridge_server",
            #     executable="rosbridge_websocket",
            #     name="rosbridge_websocket",
            #     output="screen",
            # ),
            # # rosapi node (required for /rosapi/* services)
            # Node(
            #     package="rosapi",
            #     executable="rosapi_node",
            #     name="rosapi_node",
            #     output="screen",
            # ),
        ]
    )
