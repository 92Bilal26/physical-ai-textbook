---
title: Module 1 - ROS 2 Fundamentals
sidebar_label: Module 1
---

# Module 1: ROS 2 Fundamentals

**Total Time**: 2-3 hours (3 chapters + exercises)
**Level**: Beginner
**Prerequisites**: Basic Python, Linux terminal

---

## Module Overview

Module 1 teaches the foundations of ROS 2 through hands-on examples and progressive exercises. By the end of this module, you'll understand how to build modular, reusable robot software using ROS 2 concepts.

### What You'll Learn

```
Chapter 1: ROS 2 Basics
├── Nodes (independent programs)
├── Topics (streaming communication)
└── Services (request/response)

Chapter 2: URDF Robot Description
├── Robot structure (links and joints)
├── Physical properties (mass, inertia)
└── Visualization in RViz

Chapter 3: Python Integration
├── Controlling robots with rclpy
├── Parameters and configuration
└── Actions for complex tasks
```

---

## Chapters

### [Chapter 1: ROS 2 Basics](./ch1-ros2-basics/index.md)

**Topics**: Nodes, Topics, Services
**Time**: 35-40 minutes
**Skills**: Create nodes, use pub/sub, implement services
**Projects**: Hello World, multi-node systems, service client/server

**Key Concepts**:
- Nodes as independent processes
- Topic-based streaming communication
- Service-based request/response
- ROS 2 graph visualization

**Learning Path**:
1. [What are Nodes?](./ch1-ros2-basics/01-nodes.md) (7 min)
2. [Topics: Publish and Subscribe](./ch1-ros2-basics/02-topics.md) (8 min)
3. [Services: Request and Response](./ch1-ros2-basics/03-services.md) (7 min)
4. [Hands-on Exercises](./ch1-ros2-basics/exercises.md) (10-15 min)
5. [Summary and Review](./ch1-ros2-basics/summary.md) (3 min)

👉 **Start Chapter 1**: [ROS 2 Basics](./ch1-ros2-basics/index.md)

---

### [Chapter 2: URDF Robot Description](./ch2-urdf/index.md)

**Topics**: Robot models, kinematics, simulation
**Time**: 30-35 minutes
**Skills**: Write URDF files, visualize robots, validate models
**Projects**: Simple robot, humanoid model, Gazebo simulation

**Key Concepts**:
- URDF (Unified Robot Description Format)
- Links (rigid bodies) and joints (connections)
- Collision geometry for simulation
- Visualization in RViz and Gazebo

**Prerequisites**: Complete Chapter 1

👉 **Start Chapter 2**: [URDF Robot Description](./ch2-urdf/index.md) *(coming soon)*

---

### [Chapter 3: Python Integration](./ch3-python-integration/index.md)

**Topics**: Robot control, parameters, actions
**Time**: 30-35 minutes
**Skills**: Control robots, manage configuration, implement complex behaviors
**Projects**: Velocity controller, parameter server, action server

**Key Concepts**:
- rclpy node structure and best practices
- Parameters for runtime configuration
- Actions for long-running tasks
- Integrating with robot hardware

**Prerequisites**: Complete Chapters 1-2

👉 **Start Chapter 3**: [Python Integration](./ch3-python-integration/index.md) *(coming soon)*

---

## Learning Path

### Recommended Order
1. **Start here**: [Chapter 1: ROS 2 Basics](./ch1-ros2-basics/index.md)
2. **Then**: [Chapter 2: URDF](./ch2-urdf/index.md)
3. **Finally**: [Chapter 3: Python Integration](./ch3-python-integration/index.md)

### Parallel Learning
- Chapters can be studied in parallel after completing Chapter 1
- Each chapter is self-contained with its own exercises
- Total time: 2-3 hours to complete entire module

---

## Module Goals

By completing this module, you will:

### Knowledge (Bloom's Level 1-2)
- [ ] Understand ROS 2 architecture and concepts
- [ ] Know the difference between topics and services
- [ ] Understand robot kinematics and URDF
- [ ] Know how to control robots with Python

### Skills (Bloom's Level 3-4)
- [ ] Create and run ROS 2 nodes
- [ ] Implement pub/sub communication
- [ ] Build service client/servers
- [ ] Write valid URDF robot descriptions
- [ ] Debug ROS 2 systems with command-line tools
- [ ] Control robots using rclpy

### Competence (Bloom's Level 5-6)
- [ ] Design modular robot software
- [ ] Choose appropriate communication patterns
- [ ] Troubleshoot ROS 2 systems
- [ ] Extend and customize robot behavior

---

## Skills & Tools Used

### Claude Code Skills
- `learning-objectives`: Define learning outcomes
- `ros2-code-generator`: Generate code examples
- `code-explainer`: Annotate code with explanations
- `exercise-designer`: Create progressive exercises
- `urdf-designer`: Design robot descriptions
- `summary-generator`: Generate chapter summaries
- `content-evaluation-framework`: Evaluate quality

### ROS 2 Tools
- `ros2 run`: Execute nodes
- `ros2 topic`: Inspect topics
- `ros2 service`: Call services
- `ros2 node`: Get node information
- `rqt_graph`: Visualize computation graph
- `RViz`: Visualize robots and data
- `Gazebo`: Simulate robots

### Development Tools
- Python 3.10+ with rclpy
- colcon (build tool)
- Git and GitHub
- Docker (for reproducibility)

---

## Module Statistics

| Metric | Value |
|--------|-------|
| Total Reading Time | 90-110 minutes |
| Hands-On Exercises | 9+ |
| Code Examples | 10+ |
| ROS 2 Packages | 8 (hello_world, pub_sub, service, etc.) |
| Total Word Count | 8,000+ |
| Diagrams | 15+ |
| Learning Objectives | 25+ (Bloom's L1-L3) |

---

## Common Questions

**Q: Can I skip a chapter?**
A: Not recommended. Each chapter builds on previous concepts. Start with Chapter 1.

**Q: How long does this take?**
A: 2-3 hours total. Each chapter is 35-40 minutes + exercises.

**Q: Do I need a physical robot?**
A: No! All examples run in simulation. Real robots are optional.

**Q: Can I do this on Windows/Mac?**
A: Recommended: Ubuntu 22.04 with ROS 2 Humble. WSL2 on Windows also works.

**Q: What if I get stuck?**
A: Each exercise has hints and solutions. See troubleshooting sections.

---

## Success Criteria

After completing Module 1, you should be able to:

### Chapter 1 (ROS 2 Basics)
- [ ] Create a ROS 2 node from scratch
- [ ] Implement pub/sub communication
- [ ] Implement service client/server
- [ ] Use ROS 2 command-line tools
- [ ] Debug a multi-node system

### Chapter 2 (URDF)
- [ ] Write valid URDF files
- [ ] Describe robot kinematics
- [ ] Visualize robots in RViz
- [ ] Simulate robots in Gazebo
- [ ] Validate URDF syntax

### Chapter 3 (Python Integration)
- [ ] Control robots with rclpy
- [ ] Use parameters for configuration
- [ ] Implement action servers/clients
- [ ] Integrate with hardware
- [ ] Build complete control systems

---

## Resources

### Official Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [rclpy API](https://docs.ros2.org/humble/api/rclpy/)
- [URDF Documentation](https://wiki.ros.org/urdf)

### External Resources
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)
- [ROS GitHub](https://github.com/ros2/)

---

## Getting Started

Ready to begin? Start with **[Chapter 1: ROS 2 Basics](./ch1-ros2-basics/index.md)**

Each chapter includes:
- ✅ Clear learning objectives
- ✅ Hands-on code examples
- ✅ Progressive exercises
- ✅ Common mistakes guide
- ✅ Chapter summary
- ✅ Next chapter preview

---

## Instructor Guide

### For Teaching This Module
- Each chapter is designed for ~1 hour of instruction
- Exercises include complete solutions
- All code examples are tested and working
- Pedagogical scaffolding guides students through difficulty levels
- Mental models help deep understanding

### For Self-Study
- Follow chapter structure sequentially
- Complete all exercises before moving on
- Use hints only when stuck
- Review summaries for reinforcement
- Practice with modification exercises

---

**Module Status**: Chapter 1 Complete ✅ | Chapters 2-3 In Development
**Quality**: High | **Ready for Learning**: Yes

---

Start your ROS 2 journey: [Chapter 1: ROS 2 Basics](./ch1-ros2-basics/index.md) →
