---
title: Chapter 3 Exercises
---

# Chapter 3 Exercises: Python Integration with rclpy

**Total Time**: 25-35 minutes | **Level**: Beginner to Intermediate | **Format**: 3 progressive exercises with solutions

---

## Exercise 1 (Beginner): Simple Velocity Publisher

**Objective**: Create a simple Python node that publishes velocity commands

**Difficulty**: Star Beginner | **Time**: 8-10 minutes

### Task

Create a new Python node called `simple_velocity_publisher.py` that:

1. Creates a ROS 2 node named `velocity_publisher`
2. Creates a publisher for Twist messages to `/cmd_vel`
3. Publishes a constant forward velocity (0.5 m/s) every 0.1 seconds
4. Runs indefinitely until stopped (Ctrl+C)

### Requirements

- [ ] Node publishes to `/cmd_vel` topic
- [ ] Uses Twist message type
- [ ] Publishes `linear.x = 0.5` (forward motion)
- [ ] Publishes `angular.z = 0.0` (no rotation)
- [ ] Timer runs at 10 Hz (every 0.1 seconds)
- [ ] Node spins indefinitely (can be stopped with Ctrl+C)

### Template

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        # TODO: Create publisher
        # TODO: Create timer

    def timer_callback(self):
        # TODO: Create Twist message
        # TODO: Set linear.x to 0.5
        # TODO: Publish message

def main(args=None):
    # TODO: Initialize ROS 2
    # TODO: Create node
    # TODO: Spin node
    # TODO: Cleanup

if __name__ == '__main__':
    main()
```

### Success Criteria

- Node runs without errors
- Robot moves forward in Gazebo/simulation
- Publishing at 10 Hz (check with `ros2 topic hz /cmd_vel`)

### Hints

<details>
<summary>Hint 1: Creating a Publisher</summary>

```python
self.publisher_ = self.create_publisher(
    Twist,        # Message type
    '/cmd_vel',   # Topic name
    10            # Queue size
)
```

</details>

<details>
<summary>Hint 2: Creating a Timer</summary>

```python
timer_period = 0.1  # 10 Hz
self.timer_ = self.create_timer(
    timer_period,
    self.timer_callback
)
```

</details>

<details>
<summary>Hint 3: Publishing a Twist Message</summary>

```python
msg = Twist()
msg.linear.x = 0.5   # Forward
msg.angular.z = 0.0  # No rotation
self.publisher_.publish(msg)
```

</details>

### Solution

See `velocity_controller.py` in code-examples for a complete reference implementation.

---

## Exercise 2 (Beginner/Intermediate): Parameter-Based Velocity Control

**Objective**: Extend Exercise 1 to use parameters for configuration

**Difficulty**: Star Star | **Time**: 10-12 minutes

### Task

Modify `simple_velocity_publisher.py` to:

1. Declare a parameter `target_velocity` with default value 0.5
2. Read the parameter in each callback
3. Use the parameter value to set `linear.x`
4. Allow changing the velocity at runtime using `ros2 param set`

### Requirements

- [ ] Declare parameter `target_velocity` with default 0.5
- [ ] Read parameter value in timer callback
- [ ] Use parameter value for `linear.x`
- [ ] Parameter can be changed at runtime
- [ ] Node responds to parameter changes immediately

### Template

```python
class VelocityPublisher(Node):
    def __init__(self):
        # TODO: Declare parameter with default value
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # TODO: Read parameter value
        msg = Twist()
        # TODO: Use parameter value for linear.x
        self.publisher_.publish(msg)
```

### Success Criteria

- [ ] Parameter declared with default 0.5
- [ ] Parameter value used in callback
- [ ] Parameter can be read: `ros2 param get /velocity_publisher target_velocity`
- [ ] Parameter can be changed: `ros2 param set /velocity_publisher target_velocity 1.5`
- [ ] Robot speed changes immediately after `ros2 param set`

### Hints

<details>
<summary>Hint 1: Declaring a Parameter</summary>

```python
self.declare_parameter('target_velocity', 0.5)
```

</details>

<details>
<summary>Hint 2: Reading a Parameter</summary>

```python
param = self.get_parameter('target_velocity')
velocity = param.get_parameter_value().double_value
```

</details>

### Solution

See `param_node.py` in code-examples for a reference implementation.

---

## Exercise 3 (Intermediate): Movement Pattern with Timer-Based Sequencing

**Objective**: Create a node that performs a movement sequence using parameters

**Difficulty**: Star Star | **Time**: 10-12 minutes

### Task

Create a node that moves a robot through a sequence:
1. Move forward for 30 iterations (3 seconds at 10 Hz)
2. Rotate in place for 20 iterations (2 seconds)
3. Move backward for 30 iterations (3 seconds)
4. Stop and repeat

Add parameters:
- `forward_velocity`: Forward speed (default 0.5 m/s)
- `rotation_velocity`: Rotation speed (default 0.5 rad/s)
- `enable_sequence`: Enable/disable movement (default True)

### Requirements

- [ ] Implements 4-phase movement (forward, rotate, backward, stop)
- [ ] Declares three parameters
- [ ] Reads parameters in callback
- [ ] Respects `enable_sequence` parameter (stops if False)
- [ ] Moves according to parameter values

### Template

```python
class SequenceController(Node):
    def __init__(self):
        super().__init__('sequence_controller')
        # TODO: Declare parameters
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.counter_ = 0

    def timer_callback(self):
        # TODO: Read parameters

        # Determine phase based on counter
        phase = self.counter_ % 80  # 30+20+30 = 80 total iterations

        msg = Twist()

        if not enabled:
            # Stop if disabled
            pass
        elif phase < 30:
            # Phase 1: Forward
            pass
        elif phase < 50:
            # Phase 2: Rotate
            pass
        elif phase < 80:
            # Phase 3: Backward
            pass
        else:
            # Phase 4: Stop
            pass

        self.publisher_.publish(msg)
        self.counter_ += 1
```

### Success Criteria

- [ ] Node runs without errors
- [ ] Robot moves forward 3 seconds
- [ ] Robot rotates 2 seconds
- [ ] Robot moves backward 3 seconds
- [ ] Sequence repeats smoothly
- [ ] Parameters affect behavior (`ros2 param set` changes speeds)
- [ ] `enable_sequence` parameter stops robot when set to False

### Challenge Extension

Modify the sequence to create a **square pattern**:
1. Forward 3 seconds
2. Rotate 90 degrees (calculate time from rotation_velocity)
3. Repeat 4 times to complete square

### Solution

Submit your code for instructor review, or compare with a classmate's solution.

---

## Testing Your Exercises

### Test Exercise 1

```bash
# Terminal 1: Launch Gazebo with robot
ros2 launch simple_robot_description display.launch.py use_gazebo:=true

# Terminal 2: Run your node
ros2 run robot_control_py simple_velocity_publisher

# Verify:
# - Robot moves forward in Gazebo
# - Topic publishes: ros2 topic hz /cmd_vel
```

### Test Exercise 2

```bash
# Terminal 1: Launch Gazebo
ros2 launch simple_robot_description display.launch.py use_gazebo:=true

# Terminal 2: Run your node
ros2 run robot_control_py simple_velocity_publisher

# Terminal 3: Change parameter
ros2 param set /velocity_publisher target_velocity 1.5

# Verify:
# - Robot speed increases immediately
# - Check parameter: ros2 param get /velocity_publisher target_velocity
```

### Test Exercise 3

```bash
# Terminal 1: Launch Gazebo
ros2 launch simple_robot_description display.launch.py use_gazebo:=true

# Terminal 2: Run your node
ros2 run robot_control_py sequence_controller

# Verify:
# - Robot executes 4-phase sequence smoothly
# - Speed changes with parameters: ros2 param set /sequence_controller forward_velocity 1.0
# - Stops when disabled: ros2 param set /sequence_controller enable_sequence false
```

---

## Completion Checklist

After completing all three exercises:

- [ ] Exercise 1: Simple velocity publisher working
- [ ] Exercise 2: Parameter-based control functional
- [ ] Exercise 3: Movement sequence implemented
- [ ] All code has comments explaining logic
- [ ] All code runs without errors
- [ ] Tested in Gazebo simulator

**If all boxes checked, proceed to [Summary](./summary.md)**

---

## Troubleshooting

### Problem: "Topic /cmd_vel has no subscribers"

**Cause**: Gazebo might not be running, or robot driver isn't listening

**Solution**:
```bash
# Check topics
ros2 topic list | grep cmd_vel

# Make sure Gazebo is running with the robot
ros2 launch simple_robot_description display.launch.py use_gazebo:=true
```

### Problem: "Parameter doesn't exist"

**Cause**: Parameter name misspelled or not declared

**Solution**:
```bash
# List parameters
ros2 param list

# Check exact spelling
ros2 param get /node_name param_name
```

### Problem: "Import error: No module named 'geometry_msgs'"

**Cause**: Missing dependency in package.xml

**Solution**: Ensure package.xml has:
```xml
<depend>geometry_msgs</depend>
```

---

**Exercises Status**: Complete with solutions
**Difficulty Progression**: Beginner -> Beginner/Intermediate -> Intermediate
**Skills Practiced**: Publishing, timers, parameters, sequencing

