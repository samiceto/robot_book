#!/usr/bin/env python3
"""
Long Computation Action Server - Simulates long-running task with feedback

Part of Chapter 3: ROS 2 Fundamentals
Physical AI & Humanoid Robotics Textbook
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci
import time


class LongComputationActionServer(Node):
    def __init__(self):
        super().__init__('long_computation_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            '/compute_trajectory',
            self.execute_callback
        )

        self.get_logger().info('Action server ready at /compute_trajectory')

    def execute_callback(self, goal_handle):
        """
        Execute the goal and publish feedback.

        Args:
            goal_handle: Handle to manage goal execution

        Returns:
            Result message
        """
        self.get_logger().info(f'Executing goal: order={goal_handle.request.order}')

        # Simulate 5-second computation
        total_duration = 5.0
        update_rate = 10  # Hz
        num_updates = int(total_duration * update_rate)

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Simulate computation with progress feedback
        for i in range(num_updates):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            # Compute next Fibonacci number (limited to goal.order)
            if len(feedback_msg.sequence) < goal_handle.request.order:
                feedback_msg.sequence.append(
                    feedback_msg.sequence[-1] + feedback_msg.sequence[-2]
                )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            percent_complete = (i + 1) / num_updates * 100
            self.get_logger().info(f'Progress: {percent_complete:.1f}%')

            time.sleep(1.0 / update_rate)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'Goal succeeded with result: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LongComputationActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
