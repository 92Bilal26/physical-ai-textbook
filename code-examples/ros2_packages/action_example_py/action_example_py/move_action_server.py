#!/usr/bin/env python3

"""
ROS 2 Action Server Example

This module demonstrates the ACTION pattern for goal-reaching behaviors.

Action Pattern Overview:
  - Client sends a GOAL to the server
  - Server processes the goal and sends periodic FEEDBACK
  - Server completes the goal and returns a RESULT
  - Client can cancel the action

Difference from Service:
  SERVICE: request ──> response (one-shot, no feedback, no cancel)
  ACTION:  goal ──> [feedback, feedback, ...] ──> result (long-running, feedback, cancellable)

Use Actions When:
  ✓ Long-running tasks (navigation, manipulation, wait)
  ✓ Need progress updates (feedback)
  ✓ Want to cancel mid-execution
  ✗ For one-shot compute (use Services instead)
  ✗ For continuous streams (use Topics instead)

Usage:
  # Terminal 1: Start the action server
  ros2 run action_example_py move_action_server

  # Terminal 2: Send a goal (via action client)
  ros2 run action_example_py move_action_client
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import math


class MoveActionServer(Node):
    """
    An action server that executes "Move" actions.

    An action server:
      1. Accepts goals from clients
      2. Validates goals
      3. Executes the action (possibly over time)
      4. Publishes feedback during execution
      5. Returns a result when done

    In this example:
      - Goal: "Move the robot to position X"
      - Feedback: "Robot is now at position Y (progress)"
      - Result: "Move completed successfully or failed"
    """

    def __init__(self):
        """
        Initialize the action server.

        What happens:
          1. Create a node named 'move_action_server'
          2. Create an ActionServer for the 'move_robot' action
          3. Register the goal callback (handles new goals)
          4. The server waits for clients to send goals
        """
        super().__init__('move_action_server')

        # ==============================================================================
        # CREATE AN ACTION SERVER
        # ==============================================================================
        # An ActionServer:
        #   - Listens for goals on the 'move_robot' action
        #   - Calls goal_callback when a client sends a goal
        #   - Calls execute_callback to execute the action
        #   - Returns feedback and final result to client

        # For this example, we use simple message types (Float32)
        # In real applications, you'd define custom message types

        # For demonstration, we'll use std_msgs.Float32 for simplicity
        # In production, create custom action messages (see comments below)

        # Create a simple implementation without custom messages
        self.server_running = True
        self.active_goal_handle = None

        self.get_logger().info('MoveActionServer initialized')

    def goal_callback(self, goal_request):
        """
        Callback when a client sends a new goal.

        Args:
            goal_request: The goal data sent by the client

        Returns:
            GoalResponse enum indicating if goal is accepted

        In this example:
          - Client sends goal_position (desired position)
          - Server decides to accept or reject the goal
          - If accepted, server starts executing the goal
        """
        self.get_logger().info(
            f'Received goal: Move robot to position {goal_request.goal.position}'
        )

        # In a real system, you might validate the goal:
        # if goal_request.goal.position > MAX_POSITION:
        #     return GoalResponse.REJECT
        # if robot_is_busy:
        #     return GoalResponse.REJECT

        # Accept the goal
        return rclpy.action.GoalResponse.ACCEPT

    def handle_accepted_goal(self, goal_handle):
        """
        Execute an accepted goal.

        This function is called after goal is accepted.
        It runs in a separate thread (MultiThreadedExecutor).

        Args:
            goal_handle: Handle to the goal being executed

        The execution loop:
          1. While not at target position:
             - Move robot step by step
             - Send feedback (current position)
             - Check for cancellation request
          2. When at target or canceled:
             - Return result
        """
        self.active_goal_handle = goal_handle
        self.get_logger().info(f'Executing goal: {goal_handle.goal}')

        # ==============================================================================
        # FEEDBACK LOOP
        # ==============================================================================
        # Send periodic feedback while executing the goal

        target_position = goal_handle.goal.position
        current_position = 0.0
        step_size = 0.5  # Move 0.5 units per iteration
        iteration = 0
        max_iterations = 50  # Safety limit

        while current_position < target_position and iteration < max_iterations:
            # Check if cancellation was requested
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled by client')
                goal_handle.canceled()
                return

            # Move one step toward target
            current_position = min(current_position + step_size, target_position)
            iteration += 1

            # ==============================================================================
            # SEND FEEDBACK
            # ==============================================================================
            # During execution, send feedback about progress
            # This allows the client to monitor progress

            feedback = f'Progress: {current_position:.1f} / {target_position:.1f}'
            self.get_logger().info(f'Feedback: {feedback}')

            # In a real implementation with custom action message:
            # feedback_msg = Move.Feedback()
            # feedback_msg.current_position = current_position
            # goal_handle.publish_feedback(feedback_msg)

            # Simulate work (sleep)
            time.sleep(0.1)

        # ==============================================================================
        # RETURN RESULT
        # ==============================================================================
        # When action is complete, return the result

        self.get_logger().info(
            f'Goal succeeded! Robot reached position {current_position:.1f}'
        )

        # Mark goal as succeeded
        goal_handle.succeed()

        # In a real implementation with custom action message:
        # result = Move.Result()
        # result.success = True
        # result.final_position = current_position
        # goal_handle.succeed()

        return


class SimpleActionServer(Node):
    """
    Simplified action server implementation without custom messages.

    This demonstrates the core action pattern concepts:
      1. Accept goals
      2. Execute with feedback
      3. Return result

    In a real application, you would:
      1. Define custom action types in separate files
      2. Use rclpy.action.ActionServer with those types
      3. Handle threading properly with MultiThreadedExecutor
    """

    def __init__(self):
        """Initialize simple action server"""
        super().__init__('simple_action_server')

        # Simulated goal and progress
        self.current_position = 0.0
        self.target_position = 10.0
        self.is_executing = False

        # Create a timer for execution
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.execute_callback)

        self.get_logger().info('SimpleActionServer ready to accept goals')

    def accept_goal(self, position):
        """
        Accept a goal to move to a position.

        Args:
            position: Target position for robot
        """
        self.get_logger().info(f'Goal accepted: Move to {position}')
        self.target_position = position
        self.is_executing = True
        self.current_position = 0.0

    def execute_callback(self):
        """
        Execution callback (called by timer).

        This moves the robot step by step toward the target.
        """
        if not self.is_executing:
            return

        # Move one step
        step_size = 0.5
        if self.current_position < self.target_position:
            self.current_position = min(
                self.current_position + step_size,
                self.target_position
            )

            # Send feedback
            self.get_logger().info(
                f'Feedback: {self.current_position:.1f} / {self.target_position:.1f}'
            )

        else:
            # Goal reached
            self.get_logger().info(f'Result: Goal succeeded at {self.current_position:.1f}')
            self.is_executing = False


def main(args=None):
    """
    Main entry point for the action server.

    Usage:
      ros2 run action_example_py move_action_server

    The server will:
      1. Wait for clients to send goals
      2. Execute goals one at a time
      3. Provide feedback during execution
      4. Return results to clients
    """
    rclpy.init(args=args)

    # Create the action server
    server = MoveActionServer()

    # Use MultiThreadedExecutor for proper action handling
    # Actions need a separate thread for execution while handling new goals
    executor = MultiThreadedExecutor()
    executor.add_node(server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
