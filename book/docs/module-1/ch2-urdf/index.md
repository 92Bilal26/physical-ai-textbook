---
title: Chapter 2 - URDF Robot Description
sidebar_label: Ch2 - URDF
---

# Chapter 2: URDF Robot Description

**Learning Time**: 30-35 minutes (including hands-on)
**Level**: Beginner to Intermediate
**Prerequisites**: Complete [Chapter 1: ROS 2 Basics](../ch1-ros2-basics/index.md)

---

## What is URDF and Why Should You Care?

**URDF** (Unified Robot Description Format) is the standard way to describe a robot's structure in ROS 2. Think of it as a blueprint for your robot—it tells the system:
- How many arms, legs, or links does the robot have?
- How are they connected (joints)?
- What's the shape, size, and weight of each part?
- Can motors move them? How far? How fast?

Without URDF, ROS 2 doesn't know what your robot looks like. This causes problems:
- ❌ Sensors don't know where to look
- ❌ Robots fall through the floor in simulation
- ❌ Motion planning gives unrealistic arm trajectories
- ❌ Visualization tools show nothing

With URDF:
- ✅ RViz can visualize your robot in 3D
- ✅ Gazebo can simulate physics realistically
- ✅ Motion planning respects collision constraints
- ✅ Sensor data maps to the correct 3D location
- ✅ Different robots can use the same ROS 2 code

### URDF in the ROS 2 Ecosystem

```
Your Robot Hardware
        ↓
   URDF file
        ↓
   ├─ RViz: Visualize structure & sensor data
   ├─ Gazebo: Simulate with physics
   ├─ Motion Planning: Compute safe trajectories
   ├─ Navigation: Understand robot's footprint
   └─ Kinematics: Calculate joint angles
```

### Connection to Physical AI

In this textbook:
- **Chapter 1** (ROS 2 Basics): How nodes communicate
- **Chapter 2** (this chapter): How to describe robot structure ← YOU ARE HERE
- **Chapter 3**: How to control robots using that structure

---

## What You'll Learn

This chapter covers three essential topics:

### Section 1: Links and Joints
What are the building blocks of a URDF file? How do you describe a robot's structure?
- Understand **links** (rigid bodies)
- Understand **joints** (connections between links)
- Learn URDF syntax and coordinate frames
- See a simple 2-link robot example

### Section 2: Gazebo Properties
How do you make a robot realistic for simulation? What's mass and inertia?
- Add **mass** to make gravity realistic
- Define **inertia** (resistance to rotation)
- Create **collision geometry** (prevent robots from falling through floors)
- Understand trade-offs between accuracy and performance

### Section 3: Visualizing Robots
How do you see your robot in action? What if something looks wrong?
- Load URDF in **RViz** for visualization
- Inspect **TF frames** (coordinate systems)
- Visualize **collision geometry** in Gazebo
- **Debug common URDF problems**

---

## Chapter Structure & Learning Path

| Section | Time | What You'll Do | Tools |
|---------|------|---|---|
| **Section 1: Links and Joints** | ~8 min | Understand URDF structure, create simple_robot.urdf | XML editor, `check_urdf` |
| **Section 2: Gazebo Properties** | ~9 min | Add mass, inertia, collision geometry | URDF editor |
| **Section 3: Visualizing Robots** | ~9 min | Load in RViz, debug issues, visualize humanoid.urdf | RViz, Gazebo |
| **Exercises** | ~10-15 min | Fix broken URDF, modify robots, create your own | Hands-on |
| **Summary** | ~3 min | Review key concepts and mental models | Reading |

---

## Code Examples You'll See

```python
# Launch robot visualization
ros2 launch simple_robot_description display.launch.py

# Validate URDF syntax
check_urdf simple_robot.urdf

# Visualize the TF tree (coordinate frames)
ros2 run tf2_tools view_frames
```

---

## Key Concepts at a Glance

| Concept | What It Does | Example |
|---------|-------------|---------|
| **URDF** | XML file describing robot structure | `<robot name="my_robot">` |
| **Link** | Rigid body (part of the robot) | Robot arm segment |
| **Joint** | Connection between two links | Hinge connecting arm to shoulder |
| **Revolute Joint** | Rotates around an axis (with limits) | Robot arm elbow |
| **Fixed Joint** | No motion (rigid connection) | Tool mounted on gripper |
| **Mass** | Weight of a link (kilograms) | Arm weighs 2 kg |
| **Inertia** | Resistance to spinning | Heavier parts spin slower |
| **Collision Geometry** | Physics shape (for simulation) | Box shape for arm link |
| **Visual Geometry** | Appearance (what RViz shows) | 3D mesh of robot arm |
| **Coordinate Frame** | Reference point with origin xyz, rpy | Base frame at robot center |

---

## Learning Objectives

By the end of this chapter, you will be able to:

**Remember/Understand** (L1-L2):
- [ ] Define URDF and explain why robots need descriptions
- [ ] Identify links and joints in a URDF file
- [ ] Explain the difference between a link and a joint

**Apply** (L3):
- [ ] Create valid URDF files with correct XML syntax
- [ ] Add links, joints, and physical properties to URDF
- [ ] Validate URDF files using the `check_urdf` tool

**Analyze/Evaluate** (L4-L5):
- [ ] Identify and fix broken URDF files
- [ ] Troubleshoot why a robot looks wrong in RViz or Gazebo
- [ ] Modify robot descriptions for different applications

See [Learning Objectives](./learning-objectives.md) for detailed breakdown by Bloom's taxonomy level and CEFR proficiency.

---

## Prerequisites Checklist

Before starting, make sure you have:

- [ ] Completed [Chapter 1: ROS 2 Basics](../ch1-ros2-basics/index.md)
- [ ] ROS 2 Humble installed on Ubuntu 22.04
- [ ] Text editor with syntax highlighting (VS Code recommended)
- [ ] Basic understanding of XML tags and attributes (brief intro provided)
- [ ] `check_urdf` command available: `apt install ros-humble-urdf-parser-plugin`

Not set up yet? See [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html).

---

## What You'll Build

By the end of this chapter, you'll have:

✅ **Understanding** of how URDF describes robot structure
✅ **Simple 2-link robot** - Created and visualized in RViz
✅ **Humanoid robot** - With 12 DOFs (simplified but realistic)
✅ **Debugging skills** - Can fix common URDF errors
✅ **Physics simulation ready** - Robots with inertia and collision geometry

---

## Time Breakdown

```
Total: ~35 minutes

Learning Objectives (intro):     2 min
Section 1: Links & Joints:       8 min (+ reading diagrams)
Section 2: Gazebo Properties:    9 min (+ coding)
Section 3: Visualizing Robots:   9 min (+ hands-on)
Exercises:                       10-15 min
Summary:                         3 min
```

---

## How to Use This Chapter

### For Self-Study
1. Read each section in order
2. Study the URDF examples
3. Modify examples and experiment
4. Complete all exercises
5. Check your understanding with the summary

### For Instructors
- Each section is ~8-10 minutes of lecture
- URDF examples are ready-to-use
- Exercises have solutions and common pitfalls documented
- Activities progress from Beginner to Intermediate difficulty
- Hands-on: Students validate URDF with `check_urdf` tool

### For Learners with Different Styles

**Visual Learners**:
- Study the kinematic tree diagrams
- Load robots in RViz and explore (rotate, pan, zoom)
- Use TF visualization to see coordinate frames

**Hands-On Learners**:
- Start with modifying simple_robot.urdf
- Use `check_urdf` to validate your changes
- Launch in RViz and see results immediately

**Conceptual Learners**:
- Understand the link-joint hierarchy before coding
- Read the mental models sections
- Study how ROS 2 uses URDF (frame transforms, collision checking)

---

## Next: Section 1 - Links and Joints

Ready to learn how robots are described? Let's start!

👉 **Next**: [Section 1: Links and Joints](./01-links-joints.md)

Or review the chapter overview above, then proceed section by section.

---

## Quick Links

- [Learning Objectives](./learning-objectives.md) - Detailed Bloom's taxonomy breakdown
- [Section 1: Links and Joints](./01-links-joints.md)
- [Section 2: Gazebo Properties](./02-gazebo-properties.md)
- [Section 3: Visualizing Robots](./03-visualization.md)
- [Exercises](./exercises.md) - Progressive exercises with solutions
- [Summary](./summary.md) - Key takeaways and common mistakes

---

**Chapter Status**: Ready for Learning ✅
**Last Updated**: 2025-11-30
