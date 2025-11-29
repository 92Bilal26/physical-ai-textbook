---
title: Section 2 - Working with Parameters
---

# Section 2: Working with Parameters

**Estimated Reading Time**: 8 minutes

## What are Parameters?

Parameters are configuration values stored on the ROS 2 parameter server. They allow you to:
- Configure node behavior at runtime (no recompile needed)
- Change settings without restarting the node
- Share configuration across multiple nodes

Common parameters:
- `max_velocity`: Speed limits for safety
- `control_frequency`: How often to update (Hz)
- `enable_logging`: Debug output on/off
- `robot_name`: Which robot to control

## Declaring Parameters

Before using a parameter, declare it with a default value:

```python
self.declare_parameter('max_velocity', 1.0)
self.declare_parameter('control_frequency', 10.0)
self.declare_parameter('enable_logging', True)
self.declare_parameter('robot_name', 'default_robot')
```

## Reading Parameters

Read parameter values in your code:

```python
# Read a double parameter
max_vel = self.get_parameter('max_velocity').get_parameter_value().double_value

# Read a boolean parameter
enable_log = self.get_parameter('enable_logging').get_parameter_value().bool_value

# Read a string parameter
robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
```

## Using Parameters in Code

Use parameter values to control behavior:

```python
# Clamp desired velocity to max
desired_vel = 2.0
max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
actual_vel = min(desired_vel, max_velocity)

msg = Twist()
msg.linear.x = actual_vel
self.publisher_.publish(msg)
```

## Reacting to Parameter Changes

Register a callback to respond when parameters change:

```python
self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    for param in params:
        if param.name == 'max_velocity':
            self.get_logger().info(f'Max velocity changed to {param.value}')
    return SetParametersResult(successful=True)
```

## Using Parameters from Command Line

View and modify parameters without restarting:

```bash
# List all parameters
ros2 param list

# Get a parameter value
ros2 param get /node_name max_velocity

# Set a parameter
ros2 param set /node_name max_velocity 2.5
```

## Example: Parameter-Based Controller

See `param_node.py` in code-examples for a complete example that:
1. Declares parameters with defaults
2. Reads parameters in callbacks
3. Responds to parameter changes
4. Uses parameters to limit behavior

## Benefits of Parameters

- Flexibility: Change behavior at runtime
- Reusability: Same code works for different robots
- Tuning: Adjust PID gains, speeds, without recompiling
- Safety: Limit maximum speeds without code changes

## Key Takeaways

- Parameters are configuration values on the parameter server
- Declare parameters with default values
- Read parameters using get_parameter()
- React to changes with callbacks
- Change at runtime using ros2 param set
- Use parameters to make code flexible and reusable

**Next**: [Section 3: Actions](./03-actions.md)
