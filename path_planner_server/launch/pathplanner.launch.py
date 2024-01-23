from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition


# fix: dynamic substitutions for parameters during runtime
# https://github.com/ros-planning/navigation2/issues/2117
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # package path_planner_server
    path_root = Path(get_package_share_directory("path_planner_server"))
    path_behv = path_root / "config" / "behavior_tree.config.yml"
    path_glob = path_root / "config" / "planner_server.config.yml"
    path_locl = path_root / "config" / "controller.config.yml"
    path_recv = path_root / "config" / "recovery.config.yml"
    path_fill = path_root / "config" / "filters.config.yml"
    path_rviz = path_root / "rviz" / "pathplanner.config.rviz"

    # config yaml substitutions
    new_param = {"use_sim_time" : use_sim_time}
    yaml_behv = RewrittenYaml(source_file=path_behv.as_posix(), param_rewrites=new_param)
    yaml_glob = RewrittenYaml(source_file=path_glob.as_posix(), param_rewrites=new_param)
    yaml_locl = RewrittenYaml(source_file=path_locl.as_posix(), param_rewrites=new_param)
    yaml_recv = RewrittenYaml(source_file=path_recv.as_posix(), param_rewrites=new_param)
    yaml_fill = RewrittenYaml(source_file=path_fill.as_posix(), param_rewrites=new_param)

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server_node",
                output="screen",
                parameters=[
                    yaml_glob,
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server_node",
                output="screen",
                remappings=[
                    ("/cmd_vel", "/robot/cmd_vel"),
                ],
                parameters=[
                    yaml_locl,
                ],
            ),
            Node(
                package="nav2_recoveries",
                executable="recoveries_server",
                name="recoveries_server_node",
                output="screen",
                parameters=[
                    yaml_recv,
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator_node",
                output="screen",
                parameters=[
                    yaml_behv,
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="costmap_filter_info_node",
                output="screen",
                parameters=[
                    yaml_fill,
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="sim_filter_mask_node",
                output="screen",
                parameters=[
                    yaml_fill,
                ],
                condition=IfCondition(use_sim_time),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="real_filter_mask_node",
                output="screen",
                parameters=[
                    yaml_fill,
                ],
                condition=UnlessCondition(use_sim_time),
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
                            "planner_server_node",
                            "controller_server_node",
                            "recoveries_server_node",
                            "bt_navigator_node",
                            "costmap_filter_info_node",
                            "sim_filter_mask_node",
                        ]
                    }
                ],
                condition=IfCondition(use_sim_time),
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
                            "planner_server_node",
                            "controller_server_node",
                            "recoveries_server_node",
                            "bt_navigator_node",
                            "costmap_filter_info_node",
                            "real_filter_mask_node",
                        ]
                    }
                ],
                condition=UnlessCondition(use_sim_time),
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
