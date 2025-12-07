"""
Multi-Node Launch File - Starts all Chapter 3 example nodes

Part of Chapter 3: ROS 2 Fundamentals
Physical AI & Humanoid Robotics Textbook
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to config file (if using parameters from YAML)
    # config = os.path.join(
    #     get_package_share_directory('chapter_03_ros2_basics'),
    #     'config',
    #     'params.yaml'
    # )

    return LaunchDescription([
        # Sensor simulator node
        Node(
            package='chapter_03_ros2_basics',
            executable='sensor_simulator.py',
            name='sensor_simulator',
            output='screen',
            parameters=[{
                'publish_rate': 50.0
            }]
        ),

        # Filter node
        Node(
            package='chapter_03_ros2_basics',
            executable='filter_node.py',
            name='filter_node',
            output='screen',
            parameters=[{
                'window_size': 5,
            }]
        ),

        # Calibration service
        Node(
            package='chapter_03_ros2_basics',
            executable='calibration_service.py',
            name='calibration_service',
            output='screen'
        ),

        # Action server
        Node(
            package='chapter_03_ros2_basics',
            executable='long_computation_action.py',
            name='long_computation_action_server',
            output='screen'
        ),
    ])
