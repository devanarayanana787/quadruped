import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = "spider"
    package_name = "spider"

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'worlds',
            'empty.world'
        ),
        description='Path to the world file for Gazebo Classic'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Initial X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.05', description='Initial Z position')
    roll_arg = DeclareLaunchArgument('R', default_value='0.0', description='Initial Roll')
    pitch_arg = DeclareLaunchArgument('P', default_value='0.0', description='Initial Pitch')
    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0', description='Initial Yaw')

    # Configurations
    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # Robot model path (URDF/Xacro)
    robot_model_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'spider.urdf'
    )

    rviz_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'slam.rviz'
    )

    # Process URDF
    robot_description = xacro.process_file(robot_model_path).toxml()

    # Gazebo Classic server + client launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn robot in Gazebo Classic
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    # Joint State Publisher (needed for TF tree)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        gazebo_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_model_node,
        rviz_node
    ])
