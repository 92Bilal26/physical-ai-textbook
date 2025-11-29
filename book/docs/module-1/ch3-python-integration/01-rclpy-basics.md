---
title: Section 1 - rclpy Basics
---

# Section 1: rclpy Basics and Node Structure

**Estimated Reading Time**: 8-10 minutes

## What is rclpy?

rclpy (ROS 2 Client Library for Python) is the Python interface to ROS 2. It allows you to:
- Create ROS 2 nodes in Python
- Publish and subscribe to topics
- Call and provide services
- Send and receive actions
- Use the parameter server

## Basic Node Structure

Every ROS 2 Python node follows this pattern:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## The Control Loop

You must publish velocity commands repeatedly. Use a timer to create a control loop:

- Timer period: 0.1 seconds (10 Hz)
- Callback runs every 0.1 seconds
- Publish velocity commands in the callback
- Robot receives repeated commands and moves smoothly

## Velocity Controller Example

See `velocity_controller.py` in code-examples for a complete example that:
1. Creates a publisher for Twist messages
2. Sets up a 10 Hz control loop
3. Publishes velocity commands
4. Moves the robot in patterns

## Integration with Chapter 2

The Python control code integrates with the URDF robots:

```
Python Node -> Publishes to /cmd_vel -> Gazebo -> Robot moves
```

## Key Takeaways

- rclpy is the Python interface to ROS 2
- Nodes inherit from Node class
- Publishers send messages to topics
- Timers create control loops
- Twist contains linear and angular velocity
- Control loops must run repeatedly (typically 10 Hz)

**Next**: [Section 2: Parameters](./02-parameters.md)
