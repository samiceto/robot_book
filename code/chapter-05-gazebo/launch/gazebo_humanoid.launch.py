#!/usr/bin/env python3
"""
Launch Gazebo Harmonic with humanoid robot and ROS 2 bridge.

This launch file:
1. Starts Gazebo Sim with humanoid_world.sdf
2. Spawns humanoid robot with sensors (camera, IMU)
3. Bridges sensor topics to ROS 2 (camera/image_raw, imu, joint_states)
4. Launches robot_state_publisher for TF frames

Usage:
    ros2 launch chapter-05-gazebo gazebo_humanoid.launch.py
    ros2 launch chapter-05-gazebo gazebo_humanoid.launch.py world:=obstacle_world.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('chapter_05_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='humanoid_world.sdf',
        description='Gazebo world file name (in worlds/ directory)'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI (true/false)'
    )

    # World file path
    world_file = PathJoinSubstitution([
        FindPackageShare('chapter_05_gazebo'),
        'worlds',
        LaunchConfiguration('world')
    ])

    # Gazebo Sim launch
    gazebo_args = ['-r', world_file]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ' '.join(gazebo_args),
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # ROS 2 ↔ Gazebo Bridge (sensor topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Camera
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # IMU
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
        remappings=[
            ('/camera/image_raw', '/humanoid/camera/image_raw'),
            ('/camera/camera_info', '/humanoid/camera/camera_info'),
            ('/imu', '/humanoid/imu'),
        ]
    )

    # Robot State Publisher (publishes TF frames from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(os.path.join(
                get_package_share_directory('chapter_04_urdf_sdf'),
                'urdf', 'humanoid_12dof.urdf'
            )).read()
        }]
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        gazebo,
        bridge,
        robot_state_publisher,
    ])
