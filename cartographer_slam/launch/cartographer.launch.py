from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # package cartographer_slam
    path_root = Path(get_package_share_directory("cartographer_slam"))
    path_conf = path_root / "config"
    path_rviz = path_root / "rviz" / "cartographer.config.rviz"

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
                arguments=[
                    "-configuration_directory", path_conf.as_posix(),
                    "-configuration_basename", "cartographer.config.lua",
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="occupancy_grid_node",
                name="occupancy_grid_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
                arguments=[
                    "-resolution", "0.05",
                    "-publish_period_sec", "1.0",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time
                    }
                ],
                arguments=[
                    "-d", path_rviz.as_posix(),
                ],
            ),
            LogInfo(msg="To save map file run `ros2 run nav2_map_server map_saver_cli -f <file_name>`")
        ]
    )
