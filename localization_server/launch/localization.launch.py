from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

# fix: dynamic substitutions for parameters during runtime
# https://github.com/ros-planning/navigation2/issues/2117
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    map_file = LaunchConfiguration("map_file", default="warehouse_map_sim.yaml")

    # package map_server
    path_map_server = Path(get_package_share_directory("map_server"))
    path_maps = path_map_server / "maps"

    # package localization_server
    path_root = Path(get_package_share_directory("localization_server"))
    path_amcl = path_root / "config" / "localization.config.yml"
    path_rviz = path_root / "rviz" / "localization.config.rviz"

    # config yaml substitutions
    new_param = {"use_sim_time" : use_sim_time}
    yaml_amcl = RewrittenYaml(source_file=path_amcl.as_posix(), param_rewrites=new_param)

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            DeclareLaunchArgument(name="map_file", default_value="warehouse_map_sim.yaml"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "yaml_filename": PathJoinSubstitution(
                            [
                                path_maps.as_posix(),
                                map_file
                            ]
                        ),
                    }
                ],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl_node",
                output="screen",
                parameters=[
                    yaml_amcl,
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": [
                            "map_server_node",
                            "amcl_node"
                        ]
                    }
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
        ]
    )
