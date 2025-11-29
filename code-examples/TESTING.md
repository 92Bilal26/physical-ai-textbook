# Code Examples Testing Guide

## Overview

This document describes how to test all ROS 2 code examples in this textbook.

## Prerequisites

- ROS 2 Humble installed
- Gazebo simulator installed
- Python 3.10+
- colcon build tools

## Test Procedure

### 1. Verify URDF Files

```bash
# Test simple_robot.urdf
check_urdf code-examples/ros2_packages/simple_robot_description/urdf/simple_robot.urdf

# Test humanoid.urdf
check_urdf code-examples/ros2_packages/humanoid_robot_description/urdf/humanoid.urdf
```

**Expected Output**: `OK`

### 2. Build All Python Packages

```bash
# Navigate to code examples
cd code-examples/ros2_packages

# Build all packages
colcon build

# Source install
source install/setup.bash
```

**Expected Output**: All packages build without errors

### 3. Test Individual Packages

#### Test hello_world_py
```bash
ros2 run hello_world_py hello_node
# Expected: Prints "Hello, World!" periodically
```

#### Test pub_sub_py
```bash
# Terminal 1: Start publisher
ros2 run pub_sub_py publisher

# Terminal 2: Start subscriber
ros2 run pub_sub_py subscriber

# Expected: Subscriber receives messages from publisher
```

#### Test service_example_py
```bash
# Terminal 1: Start service server
ros2 run service_example_py service_server

# Terminal 2: Call the service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Expected: Result is 8
```

#### Test robot_control_py (Velocity Controller)
```bash
# Terminal 1: Launch Gazebo with robot
ros2 launch simple_robot_description display.launch.py use_gazebo:=true

# Terminal 2: Run velocity controller
ros2 run robot_control_py velocity_controller

# Expected: Robot moves forward, then rotates
```

#### Test param_example_py (Parameters)
```bash
# Terminal 1: Run parameter node
ros2 run param_example_py param_node

# Terminal 2: View and modify parameters
ros2 param list
ros2 param get /param_node max_velocity
ros2 param set /param_node max_velocity 2.0

# Expected: Node logs parameter changes
```

#### Test action_example_py (Actions)
```bash
# Terminal 1: Start action server
ros2 run action_example_py move_action_server

# Terminal 2: Send goal via client
ros2 run action_example_py move_action_client

# Expected: Server executes goal, client receives feedback and result
```

## Test Results

- [ ] URDF validation passes (simple_robot)
- [ ] URDF validation passes (humanoid)
- [ ] All packages build without errors
- [ ] hello_world_py runs and prints messages
- [ ] pub_sub_py publisher/subscriber communicate
- [ ] service_example_py service call works
- [ ] robot_control_py controls robot in Gazebo
- [ ] param_example_py parameters work
- [ ] action_example_py actions work

## Troubleshooting

### Build Errors
- Ensure all dependencies in package.xml are installed
- Check Python version is 3.10+
- Verify ROS 2 Humble is sourced

### Runtime Errors
- Check ROS 2 is running (ros2 daemon)
- Verify Gazebo is running for velocity_controller test
- Check topic/action names match configuration

## Continuous Integration

All tests should pass before merging to main branch.
