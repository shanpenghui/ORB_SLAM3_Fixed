import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    server_address_launch_arg = DeclareLaunchArgument(
        "server_address", default_value=TextSubstitution(text=os.getenv('OPTI_TRACK_SERVER_ADDRESS', "192.168.1.12"))
    )
    client_address_launch_arg = DeclareLaunchArgument(
        "client_address", default_value=TextSubstitution(text=os.getenv('OPTI_TRACK_CLIENT_ADDRESS', ""))
    )

    opti_track_node = Node(
        package='opti_track',
        namespace=os.getenv('ROS_NAMESPACE', ''),
        executable='opti_track_client',
        name='opti_track_node',
        parameters=[{
            "server_address": LaunchConfiguration('server_address'),
            "client_address": LaunchConfiguration('client_address'),
        }],
        output='screen',
    )

    return LaunchDescription([
        server_address_launch_arg,
        client_address_launch_arg,
        opti_track_node,
    ])
