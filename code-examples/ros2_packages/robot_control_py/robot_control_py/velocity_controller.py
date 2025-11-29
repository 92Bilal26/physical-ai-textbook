#!/usr/bin/env python3

"""
Velocity Controller for ROS 2 Robots

This module demonstrates how to control a robot's velocity using rclpy.
It publishes Twist messages to the /cmd_vel topic, which controls:
  - linear.x: Forward/backward movement (m/s)
  - linear.y: Sideways movement (m/s, not common)
  - linear.z: Vertical movement (m/s, not common)
  - angular.z: Rotation (rad/s)

Key Concepts:
  1. A Node is the fundamental ROS 2 executable unit
  2. Publishers send messages to topics
  3. The control loop publishes regularly to keep robot moving
  4. Twist message is the standard velocity command in ROS 2

Usage:
  ros2 run robot_control_py velocity_controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class VelocityController(Node):
    """
    A ROS 2 Node that controls robot velocity by publishing Twist messages.

    In ROS 2:
      - Every executable is a Node
      - This Node creates a Publisher (sends messages)
      - The Publisher sends Twist messages to the /cmd_vel topic
      - The robot (or simulator) subscribes to /cmd_vel and moves accordingly
    """

    def __init__(self):
        """
        Initialize the VelocityController node.

        Args:
            None

        What happens:
          1. super().__init__('velocity_controller') creates a node named 'velocity_controller'
          2. self.publisher_ = ... creates a Publisher object
             - Publishes Twist messages (velocity commands)
             - To topic '/cmd_vel' (standard topic for velocity commands)
             - Queue size=10 means keep last 10 messages
          3. self.timer_ creates a callback that runs every 0.1 seconds (10 Hz)

        Why 10 Hz?
          - Fast enough for smooth control
          - Slow enough for network to keep up
          - Standard frequency for robot teleoperation
        """
        # Initialize the Node with name 'velocity_controller'
        super().__init__('velocity_controller')

        # Create a Publisher that:
        #   - Publishes Twist messages (velocity commands)
        #   - To the topic '/cmd_vel' (robot listens here)
        #   - With queue_size=10 (buffer capacity)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a timer that calls self.timer_callback() every 0.1 seconds
        # This creates a control loop running at 10 Hz
        timer_period = 0.1  # seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        # Initialize control state
        # This counter is used to generate different movement patterns
        self.counter_ = 0

        # Log node creation (helpful for debugging)
        self.get_logger().info('VelocityController node started')

    def timer_callback(self):
        """
        This function is called every 0.1 seconds by the timer.

        It creates and publishes a Twist message.
        The robot receives this and updates its motion.

        Key insight:
          - We must keep publishing velocity commands
          - If we publish once, the robot might move once but not continuously
          - By publishing in a loop (via timer), we maintain continuous control
        """
        # Create a new Twist message (empty, all zeros)
        msg = Twist()

        # Design a simple movement pattern:
        # - For 30 iterations: move forward
        # - For 30 iterations: rotate
        # - Repeat (period = 60 iterations × 0.1 sec = 6 seconds)

        period = 60  # Total iterations for one movement cycle
        phase = self.counter_ % period

        if phase < 30:
            # PHASE 1: Move forward
            # linear.x = forward speed in m/s
            # A wheeled robot moves forward when linear.x > 0
            msg.linear.x = 0.5  # Move forward at 0.5 m/s
            msg.angular.z = 0.0  # Don't rotate

        else:
            # PHASE 2: Rotate in place
            # angular.z = rotation speed in rad/s
            # Positive = counterclockwise, negative = clockwise
            msg.linear.x = 0.0  # Stop forward motion
            msg.angular.z = 0.5  # Rotate counterclockwise at 0.5 rad/s

        # Publish the Twist message to /cmd_vel
        # The robot (or simulator) listens to this topic and moves
        self.publisher_.publish(msg)

        # Log the published velocity (helpful for debugging)
        self.get_logger().debug(
            f'Publishing: linear.x={msg.linear.x:.2f} m/s, '
            f'angular.z={msg.angular.z:.2f} rad/s'
        )

        # Increment counter for next iteration
        self.counter_ += 1


class VelocityCommanderInteractive(Node):
    """
    Alternative implementation that allows interactive control.

    This version accepts keyboard input (from exercise) to control velocity.

    Pattern:
      1. User presses 'w' (or 'a', 's', 'd')
      2. Callback updates the commanded velocity
      3. Timer publishes the velocity at regular intervals
    """

    def __init__(self):
        """Initialize interactive velocity controller"""
        super().__init__('velocity_commander')

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Current velocity (will be modified by external input)
        self.cmd_vel_ = Twist()

        # Control loop: publish velocity every 0.1 seconds
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Interactive VelocityCommander started')

    def set_velocity(self, linear_x, angular_z):
        """
        Set the commanded velocity.

        This would be called by an external input handler (e.g., keyboard listener)
        """
        self.cmd_vel_.linear.x = linear_x
        self.cmd_vel_.angular.z = angular_z

    def timer_callback(self):
        """Publish the current commanded velocity"""
        self.publisher_.publish(self.cmd_vel_)


def main(args=None):
    """
    Main entry point for the velocity controller node.

    This function:
      1. Initializes the ROS 2 system (rclpy.init)
      2. Creates a VelocityController node
      3. Spins the node (processes callbacks indefinitely)
      4. Cleans up on exit

    Spin explanation:
      - spin() blocks forever, processing callbacks
      - Every 0.1 seconds, timer_callback() runs and publishes velocity
      - Ctrl+C to stop
    """
    # Initialize ROS 2 communication
    # This sets up the middleware for publishing/subscribing
    rclpy.init(args=args)

    # Create the node
    node = VelocityController()

    try:
        # Spin the node indefinitely
        # This keeps the node running and processing callbacks
        # Without spin(), the node would exit immediately
        rclpy.spin(node)

    except KeyboardInterrupt:
        # User pressed Ctrl+C
        pass

    finally:
        # Clean up resources
        # Stop the node and shutdown ROS 2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
