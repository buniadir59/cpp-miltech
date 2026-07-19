from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    track = LaunchConfiguration("track")
    log_level = LaunchConfiguration("log_level")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    max_distance_m = LaunchConfiguration("max_distance_m")
    config = PathJoinSubstitution([
        FindPackageShare("antidrone_turret"),
        "config",
        profile,
    ])
    track_file = PathJoinSubstitution([
        FindPackageShare("antidrone_turret"),
        "tracks",
        track,
    ])

    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="sequence.yaml"),
        DeclareLaunchArgument("track", default_value="all"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("max_distance_m", default_value="30.0"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.8"),
        Node(
            package="antidrone_turret",
            executable="target_track_publisher_node",
            parameters=[config, {"track_file": track_file}],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="antidrone_turret",
            executable="actuator_node",
            parameters=[config],
            arguments=["--ros-args", "--log-level", log_level],
        ),       
        Node(
            package="antidrone_turret",
            executable="gimbal_driver_node",
            parameters=[config],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="antidrone_turret",
            executable="yaw_servo_driver_node",
            parameters=[config],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="antidrone_turret",
            executable="turret_control_node",
            parameters=[config, {"confidence_threshold":confidence_threshold, "max_distance_m":max_distance_m}],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])
