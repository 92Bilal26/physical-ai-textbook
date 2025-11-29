# Physical AI Textbook

A comprehensive, free, open-source textbook for learning robotics with ROS 2 Humble.

**Live Website**: https://92bilal26.github.io/physical-ai-textbook/

## Overview

Physical AI Textbook teaches the complete pipeline for building and controlling robots:

1. **Module 1: ROS 2 Fundamentals** (Complete ✅)
   - Chapter 1: ROS 2 Basics (nodes, topics, services)
   - Chapter 2: URDF Robot Description (links, joints, physics)
   - Chapter 3: Python Integration (control with rclpy)

2. **Module 2: Advanced Topics** (Planned)
3. **Module 3: Control Algorithms** (Planned)

## Features

✅ **Interactive Learning**: Progressive chapters with clear learning objectives
✅ **Hands-On Exercises**: 9 exercises with solutions (3 per chapter)
✅ **Code Examples**: 8+ ROS 2 Python packages ready to run
✅ **Simulation**: URDF robots in Gazebo simulator
✅ **Free & Open**: MIT License, contributions welcome
✅ **Modern**: ROS 2 Humble best practices

## Quick Start

### 1. Install ROS 2 Humble

```bash
# Ubuntu 22.04
sudo apt-get update
sudo apt-get install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

See [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### 2. Clone Repository

```bash
git clone https://github.com/92Bilal26/physical-ai-textbook.git
cd physical-ai-textbook
```

### 3. Build Code Examples

```bash
cd code-examples/ros2_packages
colcon build
source install/setup.bash
```

### 4. Start Learning

Visit: https://92bilal26.github.io/physical-ai-textbook/

Or build locally:

```bash
cd book
npm install
npm run build
npm run serve  # http://localhost:3000
```

## Project Structure

```
physical-ai-textbook/
├── README.md                      # This file
├── book/                          # Docusaurus documentation
│   ├── docs/
│   │   ├── intro.md              # Homepage
│   │   └── module-1/             # Module 1 content
│   │       ├── ch1-ros2-basics/   # Chapter 1
│   │       ├── ch2-urdf/          # Chapter 2
│   │       └── ch3-python-integration/  # Chapter 3
│   ├── docusaurus.config.js      # Site configuration
│   └── package.json              # npm dependencies
│
├── code-examples/                # ROS 2 code examples
│   ├── ros2_packages/            # Python packages
│   │   ├── hello_world_py/
│   │   ├── pub_sub_py/
│   │   ├── service_example_py/
│   │   ├── robot_control_py/     # Velocity controller
│   │   ├── param_example_py/     # Parameters
│   │   └── action_example_py/    # Actions
│   └── TESTING.md                # Testing guide
│
└── specs/                         # Project specifications
    └── 001-docusaurus-textbook/  # Design documents
```

## Learning Path

### Module 1: ROS 2 Fundamentals (3-4 hours)

**Chapter 1: ROS 2 Basics** (40 min)
- Understand ROS 2 architecture
- Learn nodes, topics, services
- Use ROS 2 command-line tools
- Complete 3 exercises

**Chapter 2: URDF Robot Description** (40 min)
- Design robot structure with URDF
- Define links and joints
- Add physics properties (mass, inertia)
- Visualize in RViz and Gazebo
- Complete 3 exercises

**Chapter 3: Python Integration** (40 min)
- Write ROS 2 Python nodes
- Publish velocity commands
- Use parameter server
- Implement actions
- Complete 3 exercises

## Documentation

- **[Quickstart Guide](./specs/001-docusaurus-textbook/quickstart.md)** - Setup instructions
- **[Code Testing Guide](./code-examples/TESTING.md)** - How to test examples
- **[Contributing Guide](./CONTRIBUTING.md)** - How to contribute

## Chapters Completed

| Chapter | Status | Learning Objectives | Code Examples | Exercises |
|---------|--------|--------------------|-|-|
| 1: ROS 2 Basics | ✅ Complete | 5 | 3 | 3 |
| 2: URDF | ✅ Complete | 5 | 2+ | 3 |
| 3: Python Integration | ✅ Complete | 5 | 3+ | 3 |

## Code Examples Included

- `hello_world_py` - Basic ROS 2 node
- `pub_sub_py` - Publish/subscribe pattern
- `service_example_py` - Request/response pattern
- `robot_control_py` - Velocity controller
- `param_example_py` - Parameter server usage
- `action_example_py` - Action pattern
- `simple_robot_description` - 2-link robot URDF
- `humanoid_robot_description` - Humanoid robot URDF

All examples:
- Build with `colcon build`
- Run with `ros2 run`
- Work with Gazebo simulator
- Include detailed comments

## Requirements

### Minimum
- **OS**: Ubuntu 22.04 LTS or WSL2
- **ROS 2**: Humble
- **Python**: 3.10+
- **RAM**: 4 GB
- **Storage**: 10 GB

### Recommended
- **OS**: Ubuntu 22.04 LTS native
- **RAM**: 8+ GB
- **GPU**: For Gazebo simulation

## Installation

See [Quickstart Guide](./specs/001-docusaurus-textbook/quickstart.md)

## Development

### Build Documentation
```bash
cd book
npm install
npm run build
```

### Run Locally
```bash
cd book
npm run serve  # http://localhost:3000
```

### Test Code Examples
See [TESTING.md](./code-examples/TESTING.md)

## Contributing

Contributions are welcome! See [CONTRIBUTING.md](./CONTRIBUTING.md)

## License

MIT License - Feel free to use for educational purposes

## Support

- **Documentation**: https://92bilal26.github.io/physical-ai-textbook/
- **GitHub Issues**: Report bugs or feature requests
- **ROS 2 Forum**: https://discourse.ros.org/

## Citation

If you use this textbook in research or education:

```bibtex
@book{physicaltextbook2025,
  title={Physical AI Textbook: Learn Robotics with ROS 2},
  author={Physical AI Team},
  year={2025},
  publisher={GitHub},
  url={https://github.com/92Bilal26/physical-ai-textbook}
}
```

## Acknowledgments

Built with:
- ROS 2 Humble
- Docusaurus 3
- Gazebo Simulator
- Python rclpy

## Status

**Current Release**: v1.0 (Module 1 Complete)
**Last Updated**: 2025-11-30

### Roadmap

- ✅ Module 1: ROS 2 Fundamentals
- 🔄 Module 2: Advanced Topics (Planning)
- ⏳ Module 3: Control Algorithms (Future)
- ⏳ Real Robot Deployment (Future)

---

**Ready to learn robotics?** Start here: https://92bilal26.github.io/physical-ai-textbook/
