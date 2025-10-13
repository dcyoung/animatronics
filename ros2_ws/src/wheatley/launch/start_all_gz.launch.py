from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
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
            # Launch Gazebo Fortress (server-only for macOS compatibility)
            ExecuteProcess(
                cmd=['ign', 'gazebo', '-s', '-v', '4', '-r'],
                output='screen',
            ),
            # Spawn robot in Gazebo Fortress
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_entity",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "wheatley_robot",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.1",
                ],
            ),
            # Bridge between ROS and Gazebo Fortress for joint states
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_bridge",
                arguments=[
                    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                ],
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
        ]
    )
