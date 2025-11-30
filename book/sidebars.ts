import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Physical AI Textbook Sidebar Configuration
 *
 * Defines the structure of documentation navigation.
 * Shows Module 1 content with no Docusaurus tutorial pages.
 */

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: '🏠 Home',
    },
    {
      type: 'category',
      label: '📚 Module 1: ROS 2 Fundamentals',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-1/index',
          label: 'Module Overview',
        },
        {
          type: 'category',
          label: '🤖 Chapter 1: ROS 2 Basics',
          items: [
            'module-1/ch1-ros2-basics/index',
            'module-1/ch1-ros2-basics/learning-objectives',
            'module-1/ch1-ros2-basics/nodes',
            'module-1/ch1-ros2-basics/topics',
            'module-1/ch1-ros2-basics/services',
            'module-1/ch1-ros2-basics/exercises',
            'module-1/ch1-ros2-basics/summary',
          ],
        },
        {
          type: 'category',
          label: '🦾 Chapter 2: URDF Robot Description',
          items: [
            'module-1/ch2-urdf/index',
            'module-1/ch2-urdf/learning-objectives',
            'module-1/ch2-urdf/links-joints',
            'module-1/ch2-urdf/gazebo-properties',
            'module-1/ch2-urdf/visualization',
            'module-1/ch2-urdf/exercises',
            'module-1/ch2-urdf/summary',
          ],
        },
        {
          type: 'category',
          label: '⚙️ Chapter 3: Python Integration',
          items: [
            'module-1/ch3-python-integration/index',
            'module-1/ch3-python-integration/learning-objectives',
            'module-1/ch3-python-integration/rclpy-basics',
            'module-1/ch3-python-integration/parameters',
            'module-1/ch3-python-integration/actions',
            'module-1/ch3-python-integration/exercises',
            'module-1/ch3-python-integration/summary',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
