from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_udp = DeclareLaunchArgument("use_udp", default_value="true")
    host = DeclareLaunchArgument("host", default_value="")
    interfaces_launch_file_dir = os.path.join(
        get_package_share_directory("neato2_gazebo"), "launch"
    )

    return LaunchDescription(
        [
            use_udp,
            host,
            Node(
                package="neato_node2",
                executable="neato_node",
                name="neato_driver",
                parameters=[
                    {"use_udp": LaunchConfiguration("use_udp")},
                    {"host": LaunchConfiguration("host")},
                ],
                output="screen",
            ),
            Node(package="fix_scan", executable="fix_scan", name="fix_scan"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [interfaces_launch_file_dir, "/robot_state_publisher.py"]
                ),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser",
                arguments=[
                    "-0.1016",
                    "0",
                    "0.0889",
                    "-3.14159",
                    "0",
                    "0",
                    "base_link",
                    "base_laser_link",
                ],
            ),
        ]
    )
