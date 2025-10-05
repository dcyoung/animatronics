from pathlib import Path
from launch import LaunchDescription

# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    # Read URDF content
    with open(
        Path(get_package_share_directory("wheatley")) / "urdf" / "bot.urdf.xml", "r"
    ) as f:
        robot_desc = f.read()

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     "use_sim_time",
            #     default_value="false",
            #     description="Use simulation (Gazebo) clock if true",
            # ),
            # Robot State Publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        # "use_sim_time": use_sim_time,
                        "robot_description": robot_desc,
                    }
                ],
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
            # Hello World Node
            # Node(
            #     package="wheatley",
            #     executable="hello_world_node",
            #     name="hello_world_node",
            #     output="screen",
            # ),
            Node(
                package="wheatley",
                executable="bot_state_publisher_node",
                name="bot_state_publisher_node",
                output="screen",
            ),
        ]
    )
