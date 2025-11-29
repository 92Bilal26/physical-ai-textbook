---
title: Chapter 3 - Python Integration with rclpy
---

# Chapter 3: Python Integration with rclpy

**Controlling Robots with Python Code**

*From Robot Description to Robot Control*

---

## Chapter Overview

In **Chapter 2**, you learned to describe robot structure using URDF—defining links, joints, masses, and geometry.

In **Chapter 3**, you'll take the next step: **controlling those robots using Python code**.

This is where robotics comes alive. Instead of just describing what a robot looks like, you'll write Python programs that:
- ✅ Send velocity commands to make robots move
- ✅ Read and modify configuration parameters
- ✅ Implement goal-reaching behaviors with progress feedback
- ✅ Coordinate multiple robots and sensors

---

## What You'll Learn

### Three Key Topics

**Section 1: rclpy Basics** (8 minutes)
- Understand Python node structure
- Create publishers for velocity commands
- Implement control loops that send repeated commands

**Section 2: Working with Parameters** (7 minutes)
- Declare parameters (configuration values)
- Read parameters in code
- React to parameter changes without restarting

**Section 3: Actions for Goal-Reaching** (8 minutes)
- Understand when to use actions (vs topics/services)
- Implement action servers (execute goals)
- Send goals from action clients

---

## Prerequisites

Before starting this chapter, ensure you've completed:
- [ ] Chapter 1: ROS 2 Basics (nodes, topics, services)
- [ ] Chapter 2: URDF Robot Description (links, joints, visualization)
- [ ] Basic Python programming skills

---

**Chapter Status**: Ready for Learning ✅
**Last Updated**: 2025-11-30
**Estimated Time**: 35-40 minutes reading + 25-35 minutes practice

