#!/usr/bin/env python3

"""
Parameter Server Example for ROS 2

This module demonstrates how to use the ROS 2 parameter server to configure nodes at runtime.

Key Concepts:
  1. Parameters are configuration values stored on the parameter server
  2. Nodes declare parameters and read their values
  3. Parameters can be changed at runtime using 'ros2 param set'
  4. Nodes can react to parameter changes with callbacks

Usage:
  # Terminal 1: Start the parameter node
  ros2 run param_example_py param_node

  # Terminal 2: View parameters
  ros2 param list

  # Terminal 3: Change a parameter
  ros2 param set /param_node max_velocity 2.0

Why Parameters?
  - Configuration without recompiling code
  - Easy to tune robot behavior (max speed, PID gains, etc.)
  - Can change at runtime (no restart needed)
  - Makes code more flexible and reusable
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
import time


class ParameterExample(Node):
    """
    A ROS 2 Node that demonstrates parameter usage.

    This node:
      1. Declares parameters with default values
      2. Reads parameter values
      3. Responds to parameter changes in real-time
      4. Uses parameters to configure behavior
    """

    def __init__(self):
        """
        Initialize the ParameterExample node.

        What happens:
          1. Create a node named 'param_node'
          2. Declare parameters with sensible defaults
          3. Register a callback for parameter changes
          4. Create a publisher to demonstrate parameter usage
        """
        super().__init__('param_node')

        # ==============================================================================
        # PART 1: DECLARE PARAMETERS
        # ==============================================================================
        # Parameters are configuration values. We "declare" them to tell ROS 2
        # that this node uses these parameters.

        # Declare max_velocity parameter
        # - Name: 'max_velocity'
        # - Default value: 1.0 (m/s)
        # - This limits the maximum speed the robot can achieve
        # Why parameter? So user can change max speed without recompiling code
        self.declare_parameter('max_velocity', 1.0)

        # Declare control_frequency parameter
        # - Name: 'control_frequency'
        # - Default value: 10.0 (Hz)
        # - This controls how often we publish velocity commands
        self.declare_parameter('control_frequency', 10.0)

        # Declare enable_logging parameter
        # - Name: 'enable_logging'
        # - Default value: True
        # - This controls whether we log debug messages
        self.declare_parameter('enable_logging', True)

        # Declare robot_name parameter
        # - Name: 'robot_name'
        # - Default value: 'default_robot'
        # - String parameter (not numeric)
        self.declare_parameter('robot_name', 'default_robot')

        # ==============================================================================
        # PART 2: REGISTER PARAMETER CHANGE CALLBACK
        # ==============================================================================
        # We want to react when parameters change. Register a callback that
        # runs whenever any parameter is modified.
        # This is called "reactive parameters" - the node responds to changes.
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ==============================================================================
        # PART 3: CREATE PUBLISHER
        # ==============================================================================
        # Create a publisher for velocity commands
        # This demonstrates using parameters in real code
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # ==============================================================================
        # PART 4: CREATE TIMER
        # ==============================================================================
        # Create a timer that runs every 0.1 seconds
        # The actual frequency comes from control_frequency parameter
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        # Store the current desired velocity (to be limited by parameter)
        self.desired_velocity_ = 0.5

        self.get_logger().info('ParameterExample node started')
        self.log_current_parameters()

    def parameter_callback(self, params):
        """
        Callback function that runs when any parameter changes.

        Args:
            params: List of Parameter objects that changed

        Returns:
            SetParametersResult with successful=True (accept changes)

        This function:
          1. Detects which parameter changed
          2. Updates internal state if needed
          3. Returns success status to ROS 2

        Why callback?
          - React immediately to parameter changes
          - Don't need to restart the node
          - Useful for tuning (e.g., PID gains while running)
        """
        for param in params:
            # Check which parameter changed
            if param.name == 'max_velocity':
                # max_velocity changed
                self.get_logger().info(
                    f'Parameter max_velocity changed to {param.value}'
                )

            elif param.name == 'control_frequency':
                # control_frequency changed
                self.get_logger().info(
                    f'Parameter control_frequency changed to {param.value}'
                )

            elif param.name == 'enable_logging':
                # enable_logging changed
                self.get_logger().info(
                    f'Parameter enable_logging changed to {param.value}'
                )

            elif param.name == 'robot_name':
                # robot_name changed
                self.get_logger().info(
                    f'Parameter robot_name changed to {param.value}'
                )

        # Return success (accept all parameter changes)
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """
        Timer callback that publishes velocity commands.

        This demonstrates using parameter values in real code:
          1. Read max_velocity parameter
          2. Clamp desired velocity to max_velocity
          3. Publish the clamped velocity
        """
        # ==============================================================================
        # PART 5: READ PARAMETERS
        # ==============================================================================
        # Read parameter values from the parameter server

        # Read max_velocity parameter
        # get_parameter() returns a Parameter object
        # .get_parameter_value() extracts the actual value
        # .double_value gets the numeric value (for float parameters)
        max_vel_param = self.get_parameter('max_velocity')
        max_velocity = max_vel_param.get_parameter_value().double_value

        # Read enable_logging parameter
        enable_log_param = self.get_parameter('enable_logging')
        enable_logging = enable_log_param.get_parameter_value().bool_value

        # Read robot_name parameter (string)
        robot_name_param = self.get_parameter('robot_name')
        robot_name = robot_name_param.get_parameter_value().string_value

        # ==============================================================================
        # PART 6: USE PARAMETERS IN LOGIC
        # ==============================================================================
        # Now use the parameter values to control behavior

        # Clamp desired velocity to max_velocity
        # This demonstrates using a parameter to limit behavior
        actual_velocity = min(self.desired_velocity_, max_velocity)

        # Create and publish Twist message
        msg = Twist()
        msg.linear.x = actual_velocity

        self.publisher_.publish(msg)

        # Log if logging is enabled (demonstrate using boolean parameter)
        if enable_logging:
            self.get_logger().debug(
                f'Robot: {robot_name} | '
                f'Desired: {self.desired_velocity_:.2f} m/s | '
                f'Max: {max_velocity:.2f} m/s | '
                f'Actual: {actual_velocity:.2f} m/s'
            )

    def log_current_parameters(self):
        """Log all current parameter values (for debugging)"""
        self.get_logger().info('Current parameter values:')
        self.get_logger().info(
            f'  max_velocity: {self.get_parameter("max_velocity").value} m/s'
        )
        self.get_logger().info(
            f'  control_frequency: {self.get_parameter("control_frequency").value} Hz'
        )
        self.get_logger().info(
            f'  enable_logging: {self.get_parameter("enable_logging").value}'
        )
        self.get_logger().info(
            f'  robot_name: {self.get_parameter("robot_name").value}'
        )


class ParameterBasedVelocityController(Node):
    """
    Advanced example: A velocity controller that reads parameters.

    This demonstrates a more realistic use case where parameters
    configure the controller behavior.
    """

    def __init__(self):
        """Initialize parameter-based controller"""
        super().__init__('velocity_controller_configurable')

        # Declare parameters for controller tuning
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('acceleration_limit', 0.5)
        self.declare_parameter('command_timeout', 1.0)

        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Current velocity (ramped by acceleration limit)
        self.current_velocity_ = 0.0
        self.target_velocity_ = 0.0

        # Timer
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('ParameterBasedVelocityController started')

    def parameter_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            self.get_logger().info(f'Parameter {param.name} = {param.value}')
        return SetParametersResult(successful=True)

    def set_target_velocity(self, velocity):
        """
        External interface to set desired velocity.

        The controller will ramp to this velocity respecting acceleration limits.
        """
        max_linear = self.get_parameter('max_linear_velocity').value
        self.target_velocity_ = max(min(velocity, max_linear), -max_linear)

    def timer_callback(self):
        """
        Ramp current velocity toward target velocity.

        This demonstrates using a parameter (acceleration_limit)
        to constrain how fast the robot can accelerate.
        """
        # Read acceleration limit parameter
        accel_limit = self.get_parameter('acceleration_limit').value

        # Calculate velocity change (ramp)
        dt = 0.1  # timer period
        max_change = accel_limit * dt

        # Ramp current velocity toward target
        if self.current_velocity_ < self.target_velocity_:
            self.current_velocity_ = min(
                self.current_velocity_ + max_change,
                self.target_velocity_
            )
        else:
            self.current_velocity_ = max(
                self.current_velocity_ - max_change,
                self.target_velocity_
            )

        # Publish ramped velocity
        msg = Twist()
        msg.linear.x = self.current_velocity_
        self.publisher_.publish(msg)


def main(args=None):
    """
    Main entry point.

    Usage:
      ros2 run param_example_py param_node

    Then in another terminal:
      ros2 param list           # List all parameters
      ros2 param get /param_node max_velocity  # Read a parameter
      ros2 param set /param_node max_velocity 2.5  # Change a parameter
    """
    rclpy.init(args=args)
    node = ParameterExample()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
