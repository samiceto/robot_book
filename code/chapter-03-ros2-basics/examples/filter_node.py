#!/usr/bin/env python3
"""
Filter Node - Applies moving average filter to IMU data

Part of Chapter 3: ROS 2 Fundamentals
Physical AI & Humanoid Robotics Textbook
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from collections import deque


class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')

        # Declare parameters
        self.declare_parameter('window_size', 5)
        window_size = self.get_parameter('window_size').value

        # Create subscriber
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Create publisher for filtered data
        self.publisher_ = self.create_publisher(Imu, '/imu_filtered', 10)

        # Moving average filter buffers
        self.accel_x_buffer = deque(maxlen=window_size)
        self.accel_y_buffer = deque(maxlen=window_size)
        self.accel_z_buffer = deque(maxlen=window_size)

        self.get_logger().info(f'Filter node started with window_size={window_size}')

    def imu_callback(self, msg):
        # Add to buffers
        self.accel_x_buffer.append(msg.linear_acceleration.x)
        self.accel_y_buffer.append(msg.linear_acceleration.y)
        self.accel_z_buffer.append(msg.linear_acceleration.z)

        # Compute moving average
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = 'imu_link_filtered'

        filtered_msg.linear_acceleration.x = sum(self.accel_x_buffer) / len(self.accel_x_buffer)
        filtered_msg.linear_acceleration.y = sum(self.accel_y_buffer) / len(self.accel_y_buffer)
        filtered_msg.linear_acceleration.z = sum(self.accel_z_buffer) / len(self.accel_z_buffer)

        # Copy gyroscope and orientation (no filtering)
        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.orientation = msg.orientation

        # Publish filtered data
        self.publisher_.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
