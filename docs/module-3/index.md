---
sidebar_position: 7
---

# Module 3: The AI-Robot Brain (NVIDIA Isaac)

Welcome to Module 3 of the Physical AI & Humanoid Robotics textbook. This module covers NVIDIA Isaac, which provides the AI "brain" for robotic systems, enabling advanced perception, planning, and control capabilities.

## Overview

NVIDIA Isaac is a comprehensive robotics platform that includes simulation, navigation, manipulation, and AI tools. It provides the computational framework needed for intelligent robotic behavior, leveraging GPU acceleration for real-time AI processing.

## Topics in this Module

- [Isaac ROS Perception](./isaac-ros-perception.md)
- [Path Planning & Navigation](./path-planning-navigation.md)

## Learning Objectives

By the end of this module, you will be able to:
- Understand the NVIDIA Isaac platform architecture
- Implement perception systems using Isaac ROS
- Design path planning algorithms for robot navigation
- Integrate AI models with robotic systems

import HardwareNotes from '@site/src/components/HardwareNotes';

<HardwareNotes
  notes={[
    "RTX Workstation: NVIDIA Isaac applications benefit significantly from RTX-enabled GPUs for accelerated AI processing and perception tasks.",
    "Jetson Orin Nano: The platform is specifically designed to run efficiently on Jetson edge AI devices for deployment in autonomous robots.",
    "RealSense Integration: Isaac ROS works well with Intel RealSense cameras, providing optimized processing pipelines for depth and RGB data.",
    "Latency Management: Consider cloud training with local deployment for reduced latency in real-time applications."
  ]}
/>

## Next Steps

After completing this module, proceed to Module 4: Vision-Language-Action (VLA) to learn about multimodal AI systems.