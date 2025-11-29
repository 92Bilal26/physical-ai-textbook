---
title: Chapter 3 Learning Objectives
---

# Chapter 3 Learning Objectives: Python Integration with rclpy

**Chapter**: Python Integration with rclpy (Controlling Robots with Python)
**Module**: Module 1: ROS 2 Fundamentals
**Duration**: 30-35 minutes of instruction + 20-30 minutes of practice
**Prerequisites**: Chapter 1 (ROS 2 Basics), Chapter 2 (URDF Robot Description)
**Target Audience**: Beginner programmers learning to control robots using ROS 2

---

## Learning Objectives Overview

After completing this chapter, learners will be able to **control robots using Python code with rclpy**, understand how nodes communicate with services and parameters, and implement action-based control patterns. This chapter bridges the URDF descriptions (Chapter 2) with actual robot control code.

**Bloom's Taxonomy Alignment**: Remember (L1) → Understand (L2) → Apply (L3) → Analyze (L4) → Evaluate (L5)

**CEFR Proficiency Progression**: A1 (foundational) → A2 (elementary) → B1 (intermediate)

---

## Learning Objectives by Level

### LO-PY-001: Understand rclpy Node Architecture

**Bloom's Level**: Understand (L2)
**CEFR Level**: A2 (Elementary)
**Time to Achieve**: 5-7 minutes

**Objective**:
Learners will **identify and explain the structure of a ROS 2 Python node**, including:
- The role of `Node` class in creating publishers, subscribers, and service clients/servers
- The `spin()` and `spin_once()` patterns for processing callbacks
- How rclpy nodes integrate with the ROS 2 graph (from Chapter 1)
- The relationship between nodes and URDF robot descriptions (from Chapter 2)

**Success Criteria**:
- [ ] Can identify `Node` class initialization in provided Python code
- [ ] Can explain what `spin()` does and why it's necessary
- [ ] Can describe how a Python node publishes velocity commands to move a robot
- [ ] Can trace data flow: Python code → topic → robot motor

**Key Concepts**:
- rclpy node lifecycle (create → configure → activate → spinning)
- Publishers and subscribers in Python code
- Node parameters and configuration
- Integration with Chapter 2 URDF robot control

**Assessment Question**:
> "In the velocity_controller.py example, what would happen if we removed the `executor.spin()` line?"

**Common Misconception**:
- ❌ Think: "Publishers send data once and the robot moves forever"
- ✅ Correct: "Publishers send repeated velocity commands; robot only moves while receiving commands"

---

### LO-PY-002: Create Publishers to Control Robot Velocity

**Bloom's Level**: Apply (L3)
**CEFR Level**: B1 (Intermediate)
**Time to Achieve**: 8-10 minutes

**Objective**:
Learners will **write Python code that publishes velocity commands to control a simulated robot**, including:
- Creating a publisher with the correct topic name and message type
- Publishing `Twist` messages (linear and angular velocity)
- Managing publisher frequency (e.g., 10 Hz control loop)
- Stopping the robot by publishing zero velocity

**Success Criteria**:
- [ ] Can write code that creates a publisher for `/cmd_vel` topic
- [ ] Can construct and publish `Twist` messages with velocity values
- [ ] Can explain what linear.x and angular.z represent
- [ ] Can implement a simple control loop that publishes at fixed rate
- [ ] Can gracefully stop robot motion (publish zero velocities)

**Key Concepts**:
- ROS 2 message types: `geometry_msgs/Twist`
- Linear velocity (m/s) vs angular velocity (rad/s)
- Control loop frequency (Hz) and dt (time step)
- Publishing patterns in Python (single publish vs. loop)

**Code Pattern** (pseudo-code):
```python
# Create publisher
publisher = node.create_publisher(Twist, '/cmd_vel', 10)

# Publish velocity command
msg = Twist()
msg.linear.x = 0.5  # Move forward 0.5 m/s
msg.angular.z = 0.0  # Don't rotate
publisher.publish(msg)
```

**Assessment Question**:
> "Write the code to make a robot rotate counterclockwise at 1 rad/s without moving forward."

**Common Mistake**:
- ❌ Publish velocity once and expect robot to move forever
- ✅ Implement a control loop that publishes regularly (e.g., 10 Hz)

---

### LO-PY-003: Read Configuration from ROS 2 Parameters

**Bloom's Level**: Apply (L3)
**CEFR Level**: B1 (Intermediate)
**Time to Achieve**: 8-10 minutes

**Objective**:
Learners will **use ROS 2 parameters to configure robot control code at runtime**, including:
- Reading parameters from the parameter server using `node.get_parameter()`
- Declaring parameters with default values
- Implementing parameter callbacks to respond to runtime changes
- Using parameters to make control algorithms configurable (e.g., max velocity, control gains)

**Success Criteria**:
- [ ] Can declare a parameter with default value in Python code
- [ ] Can read parameter values using `get_parameter()`
- [ ] Can explain the difference between declaring and reading parameters
- [ ] Can handle parameter changes without restarting the node
- [ ] Can use parameter values in control algorithm (e.g., clamping velocity)

**Key Concepts**:
- Parameter naming convention: `/node_name/param_name`
- Parameter types: double, int, string, bool, array
- Parameter lifecycle: declare → read → (optionally) update
- Parameter callbacks for runtime changes
- Typical use: configuring max speeds, PID gains, topics to listen on

**Code Pattern** (pseudo-code):
```python
# Declare parameter with default
node.declare_parameter('max_velocity', 1.0)

# Read parameter
max_vel = node.get_parameter('max_velocity').get_parameter_value().double_value

# Clamp command velocity
desired_vel = 2.0  # Desired by algorithm
actual_vel = min(desired_vel, max_vel)  # Respect parameter limit
```

**Assessment Question**:
> "Why is it useful to put max_velocity in a parameter instead of hardcoding it? Name 2 scenarios."

**Common Misconception**:
- ❌ Think: "Parameters are only for constants; they can't change"
- ✅ Correct: "Parameters can be updated at runtime using `ros2 param set`"

---

### LO-PY-004: Implement Actions for Long-Running Tasks

**Bloom's Level**: Apply/Analyze (L3-L4)
**CEFR Level**: B1 (Intermediate)
**Time to Achieve**: 10-12 minutes

**Objective**:
Learners will **implement ROS 2 actions to control robot goal-reaching behaviors**, including:
- Understanding the action pattern (goal → feedback → result) vs topics/services
- Creating an action server that accepts movement goals
- Sending goals and monitoring feedback from an action client
- Handling action cancellation and timeouts

**Success Criteria**:
- [ ] Can explain when to use actions vs topics/services
- [ ] Can identify goal, feedback, and result in action code
- [ ] Can write an action server that moves robot to goal position
- [ ] Can write an action client that sends goal and waits for result
- [ ] Can handle feedback during action execution (e.g., current position)

**Key Concepts**:
- Action pattern: request (goal) → progress (feedback) → response (result)
- Action Server: receives goals, executes behavior, sends feedback, returns result
- Action Client: sends goal, receives feedback, waits for result
- Use case: Long-running behaviors (navigate to point, pick and place, etc.)
- Difference from services: Actions support progress feedback and cancellation

**Conceptual Diagram**:
```
Action Pattern (vs Service pattern):

SERVICE:                ACTION:
Client → Request       Client → Goal
Server → Response      Server ⟷ Feedback (progress)
         (waits)              Result (final)
         (no feedback)        (can cancel)
```

**Code Pattern** (pseudo-code):
```python
# Server: Accept goal, provide feedback, return result
def goal_callback(self, goal):
    while current_position < goal.position:
        # Publish feedback
        feedback = MoveToGoal.Feedback()
        feedback.current_position = self.current_position
        goal_handle.publish_feedback(feedback)

        # Move toward goal
        self.move(step_size)

    # Return result
    result = MoveToGoal.Result()
    result.success = True
    return result

# Client: Send goal, handle feedback
goal_future = action_client.send_goal_async(goal)
result_future = goal_future.result().get_result_async()
```

**Assessment Question**:
> "A robot needs to navigate to a location and report its current position every second. Should this use a service, topic subscription, or action? Why?"

**Common Misconception**:
- ❌ Think: "Actions are just services with feedback"
- ✅ Correct: "Actions are designed for goal-reaching behaviors with cancellation support"

---

### LO-PY-005: Integrate Python Control with URDF Robots

**Bloom's Level**: Analyze/Evaluate (L4-L5)
**CEFR Level**: B1 (Intermediate)
**Time to Achieve**: 7-9 minutes

**Objective**:
Learners will **connect Python control code to simulated robots (from Chapter 2)**, including:
- Understanding the control flow: Python node → ROS 2 topic → robot simulator (Gazebo)
- Publishing joint commands to multi-DOF robots
- Reading robot state (joint positions, velocities) from topics
- Evaluating control strategies for different robot morphologies (simple vs humanoid)

**Success Criteria**:
- [ ] Can identify which ROS 2 topics control a specific robot (e.g., `/cmd_vel` for velocity)
- [ ] Can write Python code that commands the simple_robot.urdf (2-link arm)
- [ ] Can explain how URDF joint limits (from Chapter 2) relate to Python control
- [ ] Can read robot state from ROS 2 topics
- [ ] Can design a simple control algorithm (e.g., move arm to position)

**Key Concepts**:
- Topic naming for robot control: `/joint_1/command`, `/cmd_vel`, etc.
- Message types for control: `JointTrajectory`, `Twist`, custom messages
- State feedback: reading joint positions, velocities from topics
- Control-actuator mapping: URDF joint name ↔ ROS 2 command topic
- Simulation vs real robot differences (latency, error)

**Integration with Chapter 2**:
- URDF defines robot structure (links, joints)
- Python code commands motion using ROS 2
- Gazebo simulates physics according to URDF properties

**Assessment Question**:
> "The simple_robot.urdf has joint_1 and joint_2. Write Python code that makes joint_1 rotate back and forth."

**Common Misconception**:
- ❌ Think: "I publish velocity and robot moves; that's all I need to know"
- ✅ Correct: "Different robots have different control interfaces; must match URDF structure"

---

## Assessment Strategy

### Formative Assessment (During Learning)
- **Knowledge Checks**: Each section has self-assessment questions
- **Hands-On Coding**: Write code snippets, test in provided exercise framework
- **Peer Explanation**: Explain concepts to a partner

### Summative Assessment (End of Chapter)
- **Exercises 1-3**: Progressive coding tasks with automated testing
  - Exercise 1 (Beginner): Publish velocity commands
  - Exercise 2 (Beginner/Intermediate): Use parameters to configure control
  - Exercise 3 (Intermediate): Implement action server for goal-reaching
- **Challenge Problem**: Integrate all concepts to control humanoid.urdf robot
- **Evaluation Rubric**: Code functionality, documentation, design clarity

### Success Metrics
- [ ] Learner completes all 3 exercises with working code
- [ ] Code follows best practices (comments, error handling, clear structure)
- [ ] Learner can explain the relationship between Chapter 2 (URDF) and Chapter 3 (control)
- [ ] Learner demonstrates understanding of topics, parameters, and actions in context

---

## Prerequisite Knowledge Checklist

Before starting Chapter 3, ensure you have:

- [ ] **From Chapter 1 (ROS 2 Basics)**:
  - [ ] Understand nodes, topics, and services
  - [ ] Can list ROS 2 topics using `ros2 topic list`
  - [ ] Know that topics carry messages

- [ ] **From Chapter 2 (URDF Robot Description)**:
  - [ ] Can identify links and joints in URDF
  - [ ] Understand joint types (revolute, continuous, prismatic)
  - [ ] Know how coordinate frames (XYZ, RPY) work
  - [ ] Can visualize robots in RViz

- [ ] **General Python Knowledge**:
  - [ ] Can write basic Python functions
  - [ ] Understand loops and callbacks (functions called repeatedly)
  - [ ] Familiar with packages/imports in Python

**If you're missing any of these, review the earlier chapters before continuing.**

---

## Learning Path & Time Allocation

```
Chapter 3: Python Integration with rclpy (Total: 35-40 minutes instruction + 25-35 min practice)

├─ Section 1: rclpy Basics (8 min reading + 10 min practice)
│  └─ LO-PY-001, LO-PY-002 (node structure, publishers, velocity control)
│
├─ Section 2: Parameters (7 min reading + 10 min practice)
│  └─ LO-PY-003 (reading, declaring, using parameters)
│
├─ Section 3: Actions (8 min reading + 10 min practice)
│  └─ LO-PY-004 (action pattern, servers, clients)
│
├─ Integration Lab (10 min)
│  └─ LO-PY-005 (connecting Python to URDF robots, Gazebo)
│
└─ Exercises (20-25 min practice)
   ├─ Exercise 1 (Beginner): Velocity controller - 8 min
   ├─ Exercise 2 (Beginner/Intermediate): Parameter-based control - 8 min
   └─ Exercise 3 (Intermediate): Action-based goal reaching - 9 min
```

---

## Key Terminology

| Term | Definition | Chapter Connection |
|------|-----------|-------------------|
| **rclpy** | ROS 2 client library for Python | Core topic |
| **Node** | Executable ROS 2 process | Chapter 1 |
| **Publisher** | Sends messages to topics | Chapter 1 |
| **Subscriber** | Receives messages from topics | Chapter 1 |
| **Parameter** | Configuration value on parameter server | New in Ch3 |
| **Action** | Goal-oriented communication pattern | New in Ch3 |
| **Twist** | ROS 2 message with linear/angular velocity | Core topic |
| **Joint Command** | Instruction to move a specific robot joint | Chapter 2 integration |
| **Feedback** | Progress updates during action execution | New in Ch3 |
| **Result** | Final outcome of action execution | New in Ch3 |
| **Gazebo** | Physics simulator for robots | Chapter 2 |
| **URDF** | Robot description format | Chapter 2 |

---

## Next Steps After Completing Chapter 3

After mastering these learning objectives, learners will be ready for:

1. **Advanced ROS 2 Topics** (Future modules):
   - Custom message types
   - Launch files and configuration
   - Real robot integration (safety, teleoperation)

2. **Practical Applications**:
   - Program robots to perform tasks
   - Implement navigation stacks
   - Design sensor-based behaviors

3. **Career Directions**:
   - Robotics software engineer
   - Autonomous systems developer
   - Research in robot control

---

## Instructor Notes

### Teaching Tips
- **LO-PY-001**: Use visual diagrams showing message flow (Python → topic → Gazebo)
- **LO-PY-002**: Emphasize the importance of control loops; static velocities don't work
- **LO-PY-003**: Show `ros2 param set` command so learners see parameters are "live"
- **LO-PY-004**: Contrast actions with services; mention real-world use (navigation, manipulation)
- **LO-PY-005**: Tie back to Chapter 2 URDF; show how structures relate to control

### Common Student Errors
1. Publishing velocity once instead of in a loop
2. Forgetting to read parameters; hardcoding values
3. Confusing action structure (which is goal vs feedback vs result)
4. Not matching Python topic names to robot description

### Assessment Differentiation
- **Advanced Learners**: Challenge them to implement PID controller or state machine
- **Struggling Learners**: Provide code templates; focus on understanding over implementation
- **Hands-on Learners**: Provide pre-built robot in Gazebo; focus on control code

---

## Standards Alignment

**Computer Science Standards**:
- Applied automation and robotics
- Scripting and programming fundamentals
- Real-time system design

**NGSS Alignment** (if applicable):
- Engineering Design Process
- Systems Thinking
- Iteration and Testing

---

**Document Status**: Complete ✅
**Last Updated**: 2025-11-30
**Version**: 1.0

