from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("ov9782_trig_sync"),
        "config",
        "trig_sync.params.yaml",
    ])

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to YAML parameter file",
    )

    trig_sync_node = Node(
        package="ov9782_trig_sync",
        executable="trig_sync_node",
        name="ov9782_trig_sync",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription([
        params_file_arg,
        trig_sync_node,
    ])
