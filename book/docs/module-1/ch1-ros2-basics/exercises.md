---
title: Chapter 1 Exercises
---

# Chapter 1 Exercises: ROS 2 Basics

**Total Time**: 15-20 minutes
**Level**: Beginner to Intermediate
**Created by**: `exercise-designer` skill

---

## Exercise 1 (Beginner): Modify the Hello Node

**Objective**: Get comfortable editing and running ROS 2 code

**Difficulty**: ⭐ Beginner | **Time**: 5-7 minutes

### Task

Modify `hello_node.py` to:
1. Change the message from "Hello, ROS 2!" to include your name
2. Change the logging level from `info()` to `warning()`
3. Change the timer interval from 1.0 second to 2.0 seconds
4. Rebuild and run the modified code

### Instructions

1. Open `code-examples/ros2_packages/hello_world_py/hello_world_py/hello_node.py` in your editor
2. Find the `timer_callback` method
3. Change `self.get_logger().info(...)` to `self.get_logger().warning(...)`
4. Change the message text
5. Find the `self.create_timer(1.0, ...)` line
6. Change `1.0` to `2.0`
7. Save the file
8. Rebuild: `colcon build --packages-select hello_world_py`
9. Run: `ros2 run hello_world_py hello_node`

### Expected Output

```
[WARN] [hello_node]: Hello, my_name! Message #0
[WARN] [hello_node]: Hello, my_name! Message #1
(after 2 seconds)
[WARN] [hello_node]: Hello, my_name! Message #2
```

### Hints

<details>
<summary>💡 Hint 1: Where to edit?</summary>
Look in the `timer_callback()` method where `self.get_logger().info()` is called.
</details>

<details>
<summary>💡 Hint 2: How to change logging level?</summary>
Replace `.info()` with `.warning()` on the same line.
</details>

<details>
<summary>💡 Hint 3: How to change timer interval?</summary>
In `__init__`, find `self.create_timer(1.0, ...)` and change `1.0` to `2.0`
</details>

### Validation Checklist

- [ ] Code builds without errors: `colcon build` succeeds
- [ ] Node runs: `ros2 run hello_world_py hello_node` starts
- [ ] Messages appear every 2 seconds (not 1 second)
- [ ] Messages use `[WARN]` level (not `[INFO]`)
- [ ] Message text includes your name

### Solution

See below for the complete solution code.

---

## Exercise 2 (Beginner/Intermediate): Create Your Own Node

**Objective**: Write a complete ROS 2 node from scratch

**Difficulty**: ⭐⭐ Beginner/Intermediate | **Time**: 7-10 minutes

### Task

Create a new ROS 2 node named `greeter_node` that:
1. Says "Hello, friend!" every 3 seconds
2. Logs the current message count
3. Stops gracefully when you press Ctrl+C

### Instructions

1. Create a new directory:
   ```bash
   mkdir -p code-examples/ros2_packages/greeter_py/greeter_py
   ```

2. Create `package.xml` with basic ROS 2 package metadata
3. Create `setup.py` with entry point for your node
4. Create `greeter_py/greeter_node.py` with:
   - A `GreeterNode` class inheriting from `Node`
   - A timer that fires every 3 seconds
   - A callback that logs the greeting
5. Build and test:
   ```bash
   colcon build --packages-select greeter_py
   source install/setup.bash
   ros2 run greeter_py greeter_node
   ```

### Expected Output

```
[INFO] [greeter_node]: Hello, friend! Count: 0
[INFO] [greeter_node]: Hello, friend! Count: 1
[INFO] [greeter_node]: Hello, friend! Count: 2
^C
[INFO] [greeter_node]: Goodbye!
```

### Hints

<details>
<summary>💡 Hint 1: Starting from scratch?</summary>
Copy the hello_world_py structure as a template, then modify it for your needs.
</details>

<details>
<summary>💡 Hint 2: How to add graceful shutdown?</summary>
In the main() function after `rclpy.shutdown()`, add a goodbye message.
</details>

<details>
<summary>💡 Hint 3: Package.xml and setup.py?</summary>
See `code-examples/ros2_packages/hello_world_py/` for examples.
</details>

### Validation Checklist

- [ ] New package created with correct structure
- [ ] `package.xml` includes dependencies
- [ ] `setup.py` has correct entry point
- [ ] Node builds without errors
- [ ] Node runs and logs messages every 3 seconds
- [ ] Ctrl+C stops the node cleanly

### Solution

See below for complete solution code.

---

## Exercise 3 (Intermediate): Two-Node System

**Objective**: Create two nodes that communicate (preparation for pub/sub)

**Difficulty**: ⭐⭐ Intermediate | **Time**: 8-12 minutes

### Task

Create two nodes:
1. **Sender Node**: Publishes numbers 1-5, one per second
2. **Receiver Node**: (Placeholder) Will receive in Chapter 1 Section 2

For now, just create the sender. In Section 2, you'll add the receiver.

### Instructions

1. Create a new package `sender_receiver_py`
2. Create `sender_node.py` that:
   - Is a ROS 2 node named `sender`
   - Publishes to topic `/numbers` (placeholder - not connected yet)
   - Sends integers 1 through 5
   - Waits 1 second between sends
3. Build and test:
   ```bash
   colcon build --packages-select sender_receiver_py
   ros2 run sender_receiver_py sender_node
   ```

### Expected Output

```
[INFO] [sender]: Sending number: 1
[INFO] [sender]: Sending number: 2
[INFO] [sender]: Sending number: 3
[INFO] [sender]: Sending number: 4
[INFO] [sender]: Sending number: 5
[INFO] [sender]: Done!
```

### Hints

<details>
<summary>💡 Hint 1: Where to start?</summary>
Start with hello_node.py structure, change the messages to numbers.
</details>

<details>
<summary>💡 Hint 2: Counting up to 5?</summary>
Modify the timer callback to check if `self.i < 5`, then increment.
</details>

<details>
<summary>💡 Hint 3: Done message?</summary>
After 5 messages, print "Done!" and you can stop the timer or node.
</details>

### Validation Checklist

- [ ] Package builds successfully
- [ ] Node runs and prints 5 numbers in sequence
- [ ] Numbers print one per second
- [ ] Node logs "Done!" at the end
- [ ] Node exits cleanly

### Solution

See below for complete solution code.

---

## Common Mistakes & Troubleshooting {#troubleshooting}

### "ModuleNotFoundError: No module named 'rclpy'"

**Problem**: Code can't find ROS 2 libraries

**Solution**:
```bash
source /opt/ros/humble/setup.bash  # Source ROS 2
colcon build                        # Build your package
source install/setup.bash           # Source build output
```

### "Node not found" error

**Problem**: `ros2 run` can't find your node

**Solution**:
1. Check package name: `ros2 pkg list | grep your_package`
2. Check entry point in `setup.py`
3. Rebuild: `colcon build`
4. Source again: `source install/setup.bash`

### Build fails with "Missing dependency"

**Problem**: `package.xml` doesn't list all dependencies

**Solution**:
1. Add to `<exec_depend>` in `package.xml`:
   ```xml
   <exec_depend>rclpy</exec_depend>
   <exec_depend>std_msgs</exec_depend>
   ```
2. Rebuild: `colcon build`

---

## Complete Solutions

### Exercise 1 Solution: Modified hello_node.py

```python
# File: hello_world_py/hello_world_py/hello_node.py
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        # Changed timer interval from 1.0 to 2.0
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Changed from .info() to .warning() and message text
        self.get_logger().warning(f'Hello, friend! Message #{self.i}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2 Solution: greeter_node.py

```python
# File: greeter_py/greeter_py/greeter_node.py
import rclpy
from rclpy.node import Node

class GreeterNode(Node):
    def __init__(self):
        super().__init__('greeter_node')
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        self.get_logger().info(f'Hello, friend! Count: {self.count}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = GreeterNode()
    rclpy.spin(node)
    node.get_logger().info('Goodbye!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 3 Solution: sender_node.py

```python
# File: sender_receiver_py/sender_receiver_py/sender_node.py
import rclpy
from rclpy.node import Node

class SenderNode(Node):
    def __init__(self):
        super().__init__('sender')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        if self.count < 5:
            self.get_logger().info(f'Sending number: {self.count + 1}')
            self.count += 1
        else:
            self.get_logger().info('Done!')
            # Could call rclpy.shutdown() here to stop

def main(args=None):
    rclpy.init(args=args)
    node = SenderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Next Steps

After completing these exercises:
✅ You understand how to create and modify ROS 2 nodes
✅ You can build and run ROS 2 packages
✅ You know how to structure ROS 2 code

Next, you'll learn how to make these nodes **communicate** via topics and services.

👉 **Next**: [Section 2: Topics - Publish and Subscribe](./02-topics.md)

---

**Exercises Status**: Ready ✅
**Skills Used**: `exercise-designer`
**Solutions Provided**: Yes ✅
