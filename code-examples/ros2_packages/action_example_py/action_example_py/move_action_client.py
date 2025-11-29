#!/usr/bin/env python3

"""
ROS 2 Action Client Example

This module demonstrates how to send goals to an action server
and receive feedback and results.

Usage:
  # Terminal 1: Start the action server
  ros2 run action_example_py move_action_server

  # Terminal 2: Send a goal using the client
  ros2 run action_example_py move_action_client
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time


class MoveActionClient(Node):
    """
    An action client that sends goals to the move_robot action.

    An action client:
      1. Sends goals to the server
      2. Waits for feedback updates
      3. Receives final result when action completes
      4. Can cancel the action
    """

    def __init__(self):
        """
        Initialize the action client.

        What happens:
          1. Create a node named 'move_action_client'
          2. Create an ActionClient for the 'move_robot' action
          3. Wait for the server to be available
        """
        super().__init__('move_action_client')

        # ==============================================================================
        # CREATE AN ACTION CLIENT
        # ==============================================================================
        # An ActionClient:
        #   - Sends goals to action servers
        #   - Receives feedback
        #   - Gets final result

        # For demonstration without custom action messages:
        self.server_ready = False

        self.get_logger().info('MoveActionClient initialized')

    def send_goal(self, position):
        """
        Send a goal to the move_robot action.

        Args:
            position: Target position to move robot to

        Returns:
            GoalHandle object that tracks goal status
        """
        self.get_logger().info(f'Sending goal: Move to position {position}')

        # In a real implementation with custom action message:
        # goal_msg = Move.Goal()
        # goal_msg.position = position

        # Send the goal
        # goal_future = self.action_client.send_goal_async(goal_msg)

        # For this demonstration, we simulate the goal process:
        self.simulate_goal_execution(position)

    def simulate_goal_execution(self, position):
        """
        Simulate the goal execution process.

        This demonstrates the full action lifecycle:
          1. Send goal
          2. Receive feedback
          3. Get result
        """
        self.get_logger().info(f'Goal sent. Target position: {position}')

        # Simulate feedback loop
        current_position = 0.0
        step_size = 0.5

        while current_position < position:
            time.sleep(0.2)
            current_position = min(current_position + step_size, position)

            # Log feedback (in real implementation, would be callbacks)
            progress_percent = (current_position / position) * 100
            self.get_logger().info(
                f'Feedback: Position {current_position:.1f} / {position:.1f} '
                f'({progress_percent:.0f}%)'
            )

        # Result received
        self.get_logger().info(f'Result: Goal succeeded! Position: {current_position:.1f}')

    def send_and_handle_goal(self, position):
        """
        Complete example of sending a goal and handling responses.

        This demonstrates:
          1. Sending a goal (send_goal_async)
          2. Handling goal acceptance (goal_future)
          3. Waiting for result (result_future)
          4. Handling feedback callbacks
        """
        self.get_logger().info(f'=== Sending Goal: Move to {position} ===')

        # ==============================================================================
        # STEP 1: SEND GOAL
        # ==============================================================================
        # goal_msg = Move.Goal()
        # goal_msg.position = position
        # goal_future = self.action_client.send_goal_async(
        #     goal_msg,
        #     feedback_callback=self.feedback_callback
        # )

        # ==============================================================================
        # STEP 2: WAIT FOR GOAL ACCEPTANCE
        # ==============================================================================
        # In a real implementation:
        # goal_future.add_done_callback(self.goal_response_callback)

        # ==============================================================================
        # STEP 3: HANDLE FEEDBACK AND RESULT
        # ==============================================================================
        # In a real implementation, callbacks handle:
        # - feedback_callback: Called each time server sends feedback
        # - goal_response_callback: Called when server accepts/rejects goal
        # - result_callback: Called when server returns result

        self.simulate_goal_execution(position)

    def feedback_callback(self, feedback_msg):
        """
        Callback called when server sends feedback.

        Args:
            feedback_msg: Feedback message from server

        This is called periodically during goal execution.
        """
        # In real implementation:
        # current_pos = feedback_msg.feedback.current_position
        # self.get_logger().info(f'Feedback: Robot at {current_pos}')
        pass

    def goal_response_callback(self, future):
        """
        Callback called when server responds to goal request.

        Args:
            future: Contains goal_handle (if accepted) or None (if rejected)

        The server decides to accept or reject the goal.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server')

        # Request the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Callback called when server returns the result.

        Args:
            future: Contains the final result

        This is called when the action completes.
        """
        result = future.result()
        # In real implementation:
        # success = result.result.success
        # final_pos = result.result.final_position
        # self.get_logger().info(f'Result: Success={success}, Final={final_pos}')


class SimpleActionClientDemo(Node):
    """
    Simplified demo of action client without custom messages.

    Shows the conceptual flow without requiring custom action definition.
    """

    def __init__(self):
        """Initialize simple action client"""
        super().__init__('simple_action_client_demo')

        # Create a timer to send goals periodically
        self.goal_index = 0
        timer_period = 5.0  # Send goal every 5 seconds
        self.timer = self.create_timer(timer_period, self.send_next_goal)

        self.get_logger().info('SimpleActionClientDemo started')
        self.get_logger().info('Will send goals to action server...')

    def send_next_goal(self):
        """Send the next goal in a sequence"""
        goal_positions = [5.0, 10.0, 7.5, 12.0]

        if self.goal_index >= len(goal_positions):
            self.goal_index = 0

        target = goal_positions[self.goal_index]
        self.goal_index += 1

        self.send_goal(target)

    def send_goal(self, position):
        """Send a goal"""
        self.get_logger().info(f'\n=== Sending Goal: Move to {position} ===')
        self.demonstrate_action_flow(position)

    def demonstrate_action_flow(self, target_position):
        """
        Demonstrate the action flow:
        1. Send goal
        2. Get feedback
        3. Receive result
        """
        self.get_logger().info(f'1. Goal sent: Move to {target_position}')

        # Simulate feedback loop
        current_position = 0.0
        for i in range(5):
            time.sleep(0.2)
            current_position = (target_position / 5.0) * (i + 1)
            self.get_logger().info(f'2. Feedback: Current position = {current_position:.1f}')

        self.get_logger().info(f'3. Result: Goal succeeded at {current_position:.1f}!')


def main(args=None):
    """
    Main entry point for the action client.

    Usage:
      ros2 run action_example_py move_action_client

    The client will:
      1. Wait for the action server to become available
      2. Send a goal
      3. Monitor feedback during execution
      4. Receive and process the result
    """
    rclpy.init(args=args)

    # Create the action client
    client = MoveActionClient()

    # Send a goal
    client.send_goal(position=10.0)

    try:
        # Keep running to receive feedback and result
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


def demo_main(args=None):
    """
    Alternative main that demonstrates repeated goal sending.
    """
    rclpy.init(args=args)

    demo = SimpleActionClientDemo()

    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Uncomment the one you want to run:
    main()
    # demo_main()
