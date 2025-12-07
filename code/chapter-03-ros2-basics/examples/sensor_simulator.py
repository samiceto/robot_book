#!/usr/bin/env python3
"""
Sensor Simulator Node - Publishes simulated IMU data at 50 Hz

Part of Chapter 3: ROS 2 Fundamentals
Physical AI & Humanoid Robotics Textbook
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np


class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')

        # Create publisher for /imu topic
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)

        # Create timer (20 ms = 50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Simulation state
        self.count = 0

        self.get_logger().info('Sensor simulator started, publishing to /imu at 50 Hz')

    def timer_callback(self):
        msg = Imu()

        # Header with timestamp
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate accelerometer (gravity + noise)
        msg.linear_acceleration.x = np.random.normal(0.0, 0.1)
        msg.linear_acceleration.y = np.random.normal(0.0, 0.1)
        msg.linear_acceleration.z = np.random.normal(9.81, 0.1)

        # Simulate gyroscope (random noise)
        msg.angular_velocity.x = np.random.normal(0.0, 0.05)
        msg.angular_velocity.y = np.random.normal(0.0, 0.05)
        msg.angular_velocity.z = np.random.normal(0.0, 0.05)

        # Orientation (set to identity quaternion)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        # Publish message
        self.publisher_.publish(msg)

        self.count += 1
        if self.count % 100 == 0:  # Log every 2 seconds
            self.get_logger().info(f'Published {self.count} IMU messages')


def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
