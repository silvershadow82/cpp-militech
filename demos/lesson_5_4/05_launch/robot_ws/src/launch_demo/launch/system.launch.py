from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    config = PathJoinSubstitution([
        FindPackageShare("launch_demo"),
        "config",
        profile,
    ])

    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="sim.yaml"),
        Node(
            package="launch_demo",
            executable="sensor_process",
            parameters=[config],
        ),
        Node(
            package="launch_demo",
            executable="controller_process",
            parameters=[config],
        ),
    ])
