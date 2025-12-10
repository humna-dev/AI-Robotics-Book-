---
sidebar_position: 1
---

# Module 1: The Robotic Nervous System (ROS 2)

Welcome to Module 1 of the Physical AI & Humanoid Robotics textbook. This module covers the fundamental concepts of ROS 2 (Robot Operating System 2), which serves as the nervous system for robotic applications.

## Overview

ROS 2 is the next-generation middleware framework for robotics applications. It provides the infrastructure for communication between different components of a robotic system, enabling complex behaviors through modular, reusable software packages.

## Topics in this Module

- [ROS 2 Nodes & Topics](./ros-nodes-topics.md)
- [URDF & Python Agent Integration](./urdf-python-agent.md)

## Learning Objectives

By the end of this module, you will be able to:
- Understand the core concepts of ROS 2 architecture
- Create and manage ROS 2 nodes and topics
- Define robot models using URDF
- Integrate Python agents with ROS 2 systems

import HardwareNotes from '@site/src/components/HardwareNotes';

<HardwareNotes
  notes={[
    "RTX Workstation: For intensive ROS 2 simulations, consider using an RTX-enabled workstation for accelerated physics simulation.",
    "Jetson Orin Nano: When deploying ROS 2 nodes to edge devices, the Jetson Orin Nano provides excellent performance for embedded robotics applications.",
    "Cloud Option: AWS RoboMaker provides cloud-based ROS 2 simulation and deployment capabilities.",
    "Latency Management: Consider cloud training with local deployment for reduced latency in real-time applications."
  ]}
/>

## Next Steps

After completing this module, proceed to Module 2: The Digital Twin (Gazebo & Unity) to learn about simulation environments.