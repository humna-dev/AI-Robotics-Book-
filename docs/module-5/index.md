---
sidebar_position: 13
---

# Module 5: Capstone Autonomous Humanoid Project

Welcome to Module 5 of the Physical AI & Humanoid Robotics textbook. This capstone module integrates all concepts learned throughout the course to design and implement an autonomous humanoid robot system.

## Overview

The capstone project brings together all the technologies and concepts covered in previous modules to create a comprehensive autonomous humanoid robot. This project serves as a practical application of ROS 2, simulation, perception, navigation, and cognitive systems working together.

## Topics in this Module

- [Simulated Humanoid](./simulated-humanoid.md)
- [Obstacle Navigation](./obstacle-navigation.md)

## Learning Objectives

By the end of this module, you will be able to:
- Integrate multiple robotic subsystems into a cohesive humanoid robot
- Implement coordinated control of multiple actuators for humanoid movement
- Design autonomous behavior combining perception, planning, and action
- Evaluate humanoid robot performance in complex scenarios
- Understand the challenges and solutions in humanoid robotics

import HardwareNotes from '@site/src/components/HardwareNotes';

<HardwareNotes
  notes={[
    "RTX Workstation: Humanoid robot simulation and control requires significant computational resources, especially for real-time physics and AI processing.",
    "Jetson Orin Nano: For physical humanoid robots, edge computing platforms like Jetson Orin Nano provide the necessary processing power for perception and control.",
    "RealSense Integration: 3D perception is crucial for humanoid robots to understand their environment and navigate safely.",
    "Latency Management: Consider cloud training with local deployment for reduced latency in real-time applications."
  ]}
/>

## Next Steps

After completing this module, you will have gained comprehensive knowledge of Physical AI & Humanoid Robotics and be prepared to tackle advanced projects in this field.