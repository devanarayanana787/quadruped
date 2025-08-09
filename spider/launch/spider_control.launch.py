#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('spider')

    urdf_file = os.path.join(pkg_share, 'urdf', 'spider.urdf')
    controller_yaml = os.path.join(pkg_share, 'config', 'spider_controller.yaml')

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": open(urdf_file).read()},
                controller_yaml
            ],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controller"],
        )
    ])
