from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # node parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            Node(
                package="attach_shelf",
                executable="approach_service_node",
                name="approach_service_node",
                output="screen",
                parameters=[
                    {"use_sim_time" : use_sim_time},
                ],
            ),
        ]
    )