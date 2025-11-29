# ROS 2 Code Examples

This directory contains all ROS 2 code examples used in the Physical AI & Humanoid Robotics textbook.

## Structure

```
ros2_packages/
├── hello_world_py/          # Chapter 1: Simple ROS 2 node
├── pub_sub_py/              # Chapter 1: Publisher/subscriber communication
├── service_example_py/       # Chapter 1: Service client/server
├── simple_robot_description/ # Chapter 2: Simple 2-link robot URDF
├── humanoid_robot_description/ # Chapter 2: Humanoid robot URDF
├── robot_control_py/        # Chapter 3: Robot velocity control
├── param_example_py/        # Chapter 3: ROS 2 parameters
├── action_example_py/       # Chapter 3: ROS 2 actions
└── README.md                # This file
```

## Prerequisites

### Required
- **ROS 2 Humble** (Ubuntu 22.04 LTS)
- **Python 3.10+**
- **colcon** build tool

### Installation

```bash
# Install ROS 2 Humble (Ubuntu 22.04)
# See: https://docs.ros.org/en/humble/Installation.html

# Install build tools
sudo apt update
sudo apt install -y ros-humble-ros-core ros-humble-colcon-common-extensions

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

## Building Examples

### Build All Packages

```bash
# From project root
colcon build
```

### Build Specific Package

```bash
colcon build --packages-select hello_world_py
```

### Clean Build

```bash
colcon clean packages
colcon build
```

## Running Examples

### Before Running

Always source the ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Run a Node

```bash
ros2 run hello_world_py hello_node
```

### Run with Launch File

```bash
ros2 launch simple_robot_description display.launch.py
```

## Examples by Chapter

### Chapter 1: ROS 2 Basics (Nodes, Topics, Services)

#### Example 1: Hello, ROS 2!
**Package**: `hello_world_py`

Simple node that publishes to console:

```bash
ros2 run hello_world_py hello_node
# Output: Publishing: "Hello, ROS 2!"
```

#### Example 2: Publisher/Subscriber
**Package**: `pub_sub_py`

Publisher sends messages, subscriber receives them:

```bash
# Terminal 1: Start publisher
ros2 run pub_sub_py publisher

# Terminal 2: Start subscriber
ros2 run pub_sub_py subscriber

# Output shows messages being published and received
```

#### Example 3: Service Client/Server
**Package**: `service_example_py`

Server responds to requests:

```bash
# Terminal 1: Start service server
ros2 run service_example_py service_server

# Terminal 2: Make a service request
ros2 service call /add_numbers my_interfaces/AddNumbers '{a: 5, b:3}'
# Output: sum: 8
```

### Chapter 2: URDF Robot Description

#### Example 1: Simple 2-Link Robot
**Package**: `simple_robot_description`

URDF file: `urdf/simple_robot.urdf`

Validate URDF:

```bash
check_urdf simple_robot_description/urdf/simple_robot.urdf
```

Visualize in RViz:

```bash
ros2 launch simple_robot_description display.launch.py
```

#### Example 2: Humanoid Robot
**Package**: `humanoid_robot_description`

URDF file: `urdf/humanoid.urdf`

Validate and visualize (same as above):

```bash
check_urdf humanoid_robot_description/urdf/humanoid.urdf
ros2 launch humanoid_robot_description display.launch.py
```

### Chapter 3: Python Integration with rclpy

#### Example 1: Robot Velocity Control
**Package**: `robot_control_py`

Control robot velocity:

```bash
ros2 run robot_control_py velocity_controller
# Publishes velocity commands to /cmd_vel topic
```

#### Example 2: ROS 2 Parameters
**Package**: `param_example_py`

Read and write parameters:

```bash
# Terminal 1: Start node
ros2 run param_example_py param_node

# Terminal 2: Set parameter
ros2 param set /param_node my_param 42

# View parameter
ros2 param get /param_node my_param
# Output: Integer value: 42
```

#### Example 3: ROS 2 Actions
**Package**: `action_example_py`

Long-running tasks with feedback:

```bash
# Terminal 1: Start action server
ros2 run action_example_py action_server

# Terminal 2: Send action goal
ros2 action send_goal /fibonacci my_interfaces/Fibonacci '{order: 10}'
```

## Testing & Validation

### Test Individual Package

```bash
colcon test --packages-select hello_world_py
```

### Test All Packages

```bash
colcon test
```

### View Test Results

```bash
colcon test-result --verbose
```

## Troubleshooting

### "Package not found" Error

**Problem**: `Package 'hello_world_py' not found`

**Solution**:
```bash
colcon build
source install/setup.bash  # Must source after building
ros2 run hello_world_py hello_node
```

### "ModuleNotFoundError" in Python

**Problem**: `ModuleNotFoundError: No module named 'rclpy'`

**Solution**:
```bash
source /opt/ros/humble/setup.bash  # Source ROS 2
colcon build
source install/setup.bash
```

### URDF Validation Fails

**Problem**: `error: Cannot find file ...`

**Solution**:
```bash
# Absolute path to URDF file
check_urdf ~/physical-ai-textbook/code-examples/ros2_packages/simple_robot_description/urdf/simple_robot.urdf
```

### Gazebo/RViz Won't Open

**Problem**: Visualization tool doesn't launch

**Solution**:
```bash
# Install visualization tools
sudo apt install -y ros-humble-rviz2 ros-humble-gazebo-ros

# Then try again
ros2 launch simple_robot_description display.launch.py
```

## Python Package Structure

Each Python package follows this structure:

```
package_name/
├── package.xml              # ROS 2 package metadata
├── setup.py                 # Python package setup
├── setup.cfg                # Package config
├── resource/package_name    # Package marker
└── package_name/
    ├── __init__.py
    ├── node.py              # Main node implementation
    └── [other modules].py
```

## Common Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo topic messages
ros2 topic echo /topic_name

# Get topic info
ros2 topic info /topic_name

# List services
ros2 service list

# Call service
ros2 service call /service_name ServiceType '{args}'

# List actions
ros2 action list

# Get node info
ros2 node info /node_name

# Check ROS 2 graph
rqt_graph
```

## Documentation

For detailed explanation of each example, see:

- **Chapter 1**: `book/docs/module-1/ch1-ros2-basics/`
- **Chapter 2**: `book/docs/module-1/ch2-urdf/`
- **Chapter 3**: `book/docs/module-1/ch3-python-integration/`

Each chapter directory contains:
- Code example explanations
- Exercise solutions
- Links to full documentation

## Contributing

When adding new examples:

1. Create package: `ros2 pkg create my_package --build-type ament_python`
2. Implement node(s) with clear comments
3. Test: `colcon build && colcon test`
4. Document in chapter README
5. Update this file with package description

## Version Information

- **ROS 2 Version**: Humble 22.04 LTS
- **Python Version**: 3.10+
- **Build Tool**: colcon
- **Package Format**: ament_python

## Resources

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **URDF Documentation**: https://wiki.ros.org/urdf
- **rclpy API**: https://docs.ros2.org/humble/api/rclpy/

## Support

For issues with examples:

1. Check the textbook chapter for context
2. See troubleshooting section above
3. File issue on GitHub
4. Check ROS 2 Answers forum

---

**Last Updated**: 2025-11-30
**Tested On**: ROS 2 Humble, Ubuntu 22.04 LTS
**Maintainer**: Physical AI Textbook Team
