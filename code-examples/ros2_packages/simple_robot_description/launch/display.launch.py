#!/usr/bin/env python3
"""
Gazebo/RViz Launch File for Simple Robot
========================================

This launch file displays the simple_robot.urdf in RViz or Gazebo.

Components:
1. robot_state_publisher: Publishes TF transforms from URDF
2. joint_state_publisher: Publishes joint angles (with GUI for manual control)
3. rviz2: 3D visualization tool

Usage:
  ros2 launch simple_robot_description display.launch.py

To visualize in Gazebo instead:
  ros2 launch simple_robot_description display.launch.py use_gazebo:=true

To load a custom RViz config:
  ros2 launch simple_robot_description display.launch.py rvizconfig:=/path/to/config.rviz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for displaying simple_robot."""

    # Get the package directory
    pkg_share = get_package_share_directory("simple_robot_description")

    # Default URDF file path
    urdf_file = os.path.join(pkg_share, "urdf", "simple_robot.urdf")

    # Verify URDF file exists
    if not os.path.isfile(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_gazebo = LaunchConfiguration("use_gazebo", default="false")
    rvizconfig = LaunchConfiguration(
        "rvizconfig",
        default=os.path.join(pkg_share, "rviz", "simple_robot.rviz")
    )

    # Load URDF file content as a parameter
    # This way robot_state_publisher has the full robot description
    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    # Package the robot description as a parameter value
    robot_description = ParameterValue(robot_desc, value_type=str)

    # =========================================================================
    # NODE 1: robot_state_publisher
    # =========================================================================
    # This node:
    # - Reads the robot description (URDF)
    # - Reads joint states from /joint_states topic
    # - Publishes TF transforms for each joint
    # - Allows RViz and Gazebo to visualize the robot correctly
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            # Optional: remap topics if needed
            # ("/joint_states", "/custom_joint_states"),
        ],
    )

    # =========================================================================
    # NODE 2: joint_state_publisher_gui
    # =========================================================================
    # This node:
    # - Publishes joint states on the /joint_states topic
    # - Provides a GUI where you can manually control joint angles
    # - Useful for testing and visualization
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # NODE 3: rviz2
    # =========================================================================
    # This node:
    # - Visualizes the robot in 3D
    # - Reads TF transforms from robot_state_publisher
    # - Reads geometry from URDF
    # - Can display sensors, frames, and other visualizations
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rvizconfig],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # =========================================================================
    # Create launch description with all nodes
    # =========================================================================
    ld = LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock"
            ),
            DeclareLaunchArgument(
                "use_gazebo",
                default_value="false",
                description="Launch with Gazebo simulator"
            ),
            DeclareLaunchArgument(
                "rvizconfig",
                default_value=rvizconfig,
                description="RViz configuration file"
            ),

            # Add nodes to launch
            robot_state_publisher_node,
            joint_state_publisher_gui,
            rviz_node,
        ]
    )

    return ld


if __name__ == "__main__":
    generate_launch_description()
