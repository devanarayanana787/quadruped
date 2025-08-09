import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package and URDF setup
    pkg_name = 'spider'  # <-- Change this to your package name
    urdf_file = 'spider1.urdf'  # <-- Change to your URDF/Xacro
    urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', urdf_file)

    return LaunchDescription([
        # Optional: choose RViz config
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(get_package_share_directory(pkg_name), 'config', 'slam.rviz'),
            description='Path to RViz config file'
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Robot State Publisher (publishes TF from URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # RViz 2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        )
    ])
