---
title: Chapter 1 - ROS 2 Basics
sidebar_label: Ch1 - ROS 2 Basics
---

# Chapter 1: ROS 2 Basics (Nodes, Topics, Services)

**Learning Time**: 25-30 minutes
**Level**: Beginner
**Prerequisites**: Basic Python, Linux terminal comfort

---

## Introduction: What is ROS 2 and Why Should You Care?

Robot Operating System 2 (ROS 2) is the middleware that powers modern robotics. Instead of writing a monolithic robot control program, ROS 2 lets you build robots as a collection of independent, communicating processes called **nodes**.

Think of a robot as a team of specialists:
- One specialist reads sensor data (camera, lidar, IMU)
- Another processes that data (perception)
- A third makes decisions (planning)
- A fourth sends commands to motors (control)

ROS 2 is the communication system that lets these specialists talk to each other reliably and efficiently.

### Key Benefits

1. **Modularity**: Write small, focused programs instead of monolithic code
2. **Reusability**: Use existing ROS 2 packages instead of reinventing the wheel
3. **Scalability**: Add more nodes and features without rewriting everything
4. **Debugging**: Inspect messages flowing through the system
5. **Hardware Independence**: Same code runs on different robots (simulator or real)

### Connection to Physical AI

In the Physical AI curriculum, ROS 2 is your foundation:
- **Chapter 1** (this chapter): How nodes communicate
- **Chapter 2**: How to describe robot structure (URDF)
- **Chapter 3**: How to control robots using these tools

---

## Chapter Structure

This chapter has 4 sections plus exercises:

1. **Nodes and the Graph** (Section 1)
   - What are nodes?
   - How do they form a computation graph?
   - Your first ROS 2 node

2. **Topics: Publish and Subscribe** (Section 2)
   - One-to-many communication
   - Decoupled systems
   - Real-world use cases

3. **Services: Request and Response** (Section 3)
   - Synchronous communication
   - When to use services vs. topics
   - Building interactive systems

4. **Putting It Together** (Section 4)
   - Multi-node systems
   - Debugging with ROS 2 tools
   - Best practices

---

## Learning Objectives

By the end of this chapter, you will be able to:

**Remember**:
- Define what a ROS 2 node is
- List ROS 2 communication patterns
- Recall command-line tools (ros2 run, ros2 topic, ros2 service)

**Understand**:
- Explain how the ROS 2 graph works
- Describe pub/sub vs. service patterns
- Understand message flow and decoupling

**Apply**:
- Create a simple publisher node
- Create a simple subscriber node
- Create a service client and server
- Debug a running system with CLI tools

See [Learning Objectives](./learning-objectives.md) for detailed breakdown by Bloom's taxonomy level.

---

## Key Concepts at a Glance

| Concept | What It Does | When to Use |
|---------|-------------|------------|
| **Node** | Independent ROS 2 program | Always - everything is a node |
| **Topic** | One-way communication channel | Sensor data, state updates (one→many) |
| **Publisher** | Sends data to a topic | Sensor drivers, data producers |
| **Subscriber** | Receives data from a topic | Data consumers, processing |
| **Service** | Request/response call | Commands, queries (synchronous) |
| **Message** | Data structure on a topic | Sensor readings, commands |

---

## Tools You'll Use

### Command Line
```bash
ros2 run [package] [node]        # Run a node
ros2 topic list                   # See all topics
ros2 topic echo /topic_name       # Watch messages
ros2 service list                 # See all services
ros2 service call /srv AddNumbers '{a: 5, b: 3}'  # Call service
rqt_graph                         # Visualize the ROS 2 graph
```

### Python Code (rclpy)
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Create publishers, subscribers, services

    def main():
        rclpy.init()
        node = MyNode()
        rclpy.spin(node)  # Event loop
        rclpy.shutdown()
```

---

## Chapter Contents

1. [Nodes and the Graph](./01-nodes.md) - 7 min
2. [Topics: Publish and Subscribe](./02-topics.md) - 8 min
3. [Services: Request and Response](./03-services.md) - 7 min
4. [Exercises](./exercises.md) - 10-15 min
5. [Summary](./summary.md) - 3 min

**Total Time**: 35-40 minutes (including hands-on)

---

## How to Use This Chapter

### For Self-Study
1. Read each section in order
2. Run the code examples yourself
3. Modify examples and experiment
4. Complete all exercises
5. Check your understanding with the summary

### For Instructors
- Each section is ~7-8 minutes of lecture
- Code examples are ready-to-run in any ROS 2 Humble environment
- Exercises have solutions and common pitfalls documented
- Activities scale from beginner to intermediate difficulty

### For Learners with Different Styles

**Visual Learners**:
- Focus on the ROS 2 graph diagrams
- Use `rqt_graph` to visualize running systems
- Study the node/topic/service relationship diagrams

**Hands-On Learners**:
- Run code examples immediately
- Modify and experiment with code
- Complete exercises as you learn

**Theoretical Learners**:
- Read detailed explanations in each section
- Study the mental models section in the summary
- Understand the "why" behind design patterns

---

## Prerequisites Checklist

Before starting, make sure you have:

- [ ] ROS 2 Humble installed on Ubuntu 22.04
- [ ] Python 3.10+ working with rclpy
- [ ] Basic understanding of Python (functions, classes)
- [ ] Linux terminal comfort level (mkdir, cd, ls)
- [ ] Ability to run `ros2 --version` and see version info

Not set up yet? See [Quickstart Guide](../../../quickstart.md).

---

## What You'll Build

By the end of this chapter, you'll have created:

1. **hello_node.py**: A simple publisher that prints "Hello, ROS 2!"
2. **publisher.py + subscriber.py**: A working pub/sub system
3. **service_server.py + service_client.py**: A request/response system
4. **3+ working exercises**: Custom nodes you build from scratch

All code examples are:
- ✅ Tested and working in ROS 2 Humble
- ✅ Fully commented with explanations
- ✅ Available in `code-examples/ros2_packages/`
- ✅ Ready to extend and modify

---

## Common Questions Answered

**Q: Do I need to understand C++?**
A: No! This chapter uses Python exclusively. C++ comes later if you need it.

**Q: Do I need a physical robot?**
A: No! All examples run in simulation. Real hardware comes in Chapter 3.

**Q: How long will this take?**
A: 30-40 minutes for reading + exercises. Faster if you skip exercises, longer if you explore.

**Q: Can I skip this chapter?**
A: Not recommended. This chapter is the foundation for everything that follows.

**Q: What if I get stuck?**
A: Each exercise has hints (collapsed sections). See [Troubleshooting](./exercises.md#troubleshooting).

---

## Next Steps

After completing this chapter:
- **Immediately**: Do the exercises and verify all code runs
- **Next Chapter**: Chapter 2 will teach you how to describe robot structure (URDF)
- **Practical Project**: Build a simple robot that uses pub/sub communication

---

## Additional Resources

### Official Documentation
- [ROS 2 Humble Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [rclpy API Reference](https://docs.ros2.org/humble/api/rclpy/)
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts/About-ROS-2.html)

### External Tutorials
- [ROS 2 Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)

---

## Chapter Statistics

| Metric | Value |
|--------|-------|
| Reading Time | 15-20 min |
| Hands-On Time | 10-15 min |
| Code Examples | 3+ (publisher, subscriber, service) |
| Exercises | 3 (beginner to intermediate) |
| Diagrams | 4+ (ROS 2 graph, message flow, architecture) |
| Key Concepts | 6 (node, topic, publisher, subscriber, service, message) |

---

## Ready to Start?

👉 **Next**: [Section 1: Nodes and the Graph](./01-nodes.md)

Or jump to a specific section:
- [Section 2: Topics](./02-topics.md)
- [Section 3: Services](./03-services.md)
- [Exercises](./exercises.md)

---

**Chapter Status**: Ready for learning ✅
**Skills Used**: `learning-objectives`
**Created**: 2025-11-30
**For Questions**: See chapter summary or troubleshooting section
