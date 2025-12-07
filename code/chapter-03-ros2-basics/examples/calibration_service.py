#!/usr/bin/env python3
"""
Calibration Service Node - Provides IMU calibration service

Part of Chapter 3: ROS 2 Fundamentals
Physical AI & Humanoid Robotics Textbook
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool


class CalibrationService(Node):
    def __init__(self):
        super().__init__('calibration_service')

        # Create service server
        self.srv = self.create_service(
            SetBool,
            '/calibrate_imu',
            self.calibrate_callback
        )

        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0

        self.get_logger().info('Calibration service ready at /calibrate_imu')

    def calibrate_callback(self, request, response):
        """
        Request: bool data (True = calibrate, False = reset)
        Response: bool success, string message
        """
        if request.data:  # Calibrate
            # Simulate calibration (in reality, would collect samples and compute mean)
            self.bias_x = 0.05
            self.bias_y = -0.03
            self.bias_z = 0.02

            response.success = True
            response.message = f'Calibration complete. Bias: x={self.bias_x}, y={self.bias_y}, z={self.bias_z}'

            self.get_logger().info(response.message)
        else:  # Reset
            self.bias_x = 0.0
            self.bias_y = 0.0
            self.bias_z = 0.0

            response.success = True
            response.message = 'Bias reset to zero'

            self.get_logger().info(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
