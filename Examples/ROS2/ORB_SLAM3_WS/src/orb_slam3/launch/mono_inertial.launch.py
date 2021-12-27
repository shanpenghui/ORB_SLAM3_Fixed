import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('orb_slam3')

    # namespace = LaunchConfiguration('namespace')
    # params_file = LaunchConfiguration('params_file')
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # log_file_prefix = LaunchConfiguration('log_file_prefix')

    # robot_localization_dir = get_package_share_directory('robot_localization')
    # robot_localization_launch_file = os.path.join(robot_localization_dir, 'launch', 'ekf.launch.py')

    # mower_driver_dir = get_package_share_directory('mower_driver')
    # mower_driver_launch_file = os.path.join(mower_driver_dir, 'launch', 'mower_driver.launch.py')

    # nav_launch_file = os.path.join(pkg_dir, 'launch', 'nav.launch.py')

    # declare_namespace_cmd = DeclareLaunchArgument(
    #     'namespace', default_value='',
    #     description='Top-level namespace')

    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(pkg_dir, 'params', 'uwb_params.yaml'),
    #     description='Full path to the ROS2 parameters file to use')

    # declare_use_sim_time_cmd = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='false',
    #     description='Use simulation (Gazebo) clock if true')

    # declare_log_file_prefix_cmd = DeclareLaunchArgument(
    #     'log_file_prefix',
    #     default_value=pkg_dir,
    #     description='Path of log file')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]

    # with open(params_file) as file:
    #     param_substitutions = yaml.load(file, Loader=yaml.FullLoader)

    # param_substitutions = {key: str(value) for key, value in param_substitutions.items()}

    # param_substitutions = {
    #     'use_sim_time': use_sim_time,
    #     'params_file': params_file,
    #     'log_file_prefix': log_file_prefix
    # }

    # configured_params = RewrittenYaml(
    #     source_file=params_file,
    #     root_key=namespace,
    #     param_rewrites=param_substitutions,
    #     convert_types=True
    # )

    # robot_localization_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(robot_localization_launch_file))

    localization_cmd = Node(
        package="orb_slam3",
        executable="mono_inertial",
        name="mono_inertial_node",
        output='log',
        namespace='',
        emulate_tty=True
        # parameters=[('--ros-args', '--log-level', 'DEBUG'), configured_params]
    )

    # mower_diver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(mower_driver_launch_file))

    # nav_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(nav_launch_file))

    ld = LaunchDescription()
    # ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_log_file_prefix_cmd)

    #ld.add_action(mower_diver_cmd)
    #ld.add_action(robot_localization_cmd)

    #ld.add_action(nav_cmd)
    ld.add_action(localization_cmd)
    return ld
