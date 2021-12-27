from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
        package="orbslam3",
        executable="orbslam3",
        arguments=[
            "/usr/local/share/ORB_SLAM3/Vocabulary/ORBvoc.txt",
            "/usr/local/share/ORB_SLAM3/Config/RealSense-D435i-RGBD.yaml",
            "RGBD"
        ]
    )
    ])
