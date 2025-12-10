---
sidebar_position: 3
---

# URDF & Python Agent Integration

This topic covers Unified Robot Description Format (URDF) and how to integrate Python agents with robotic systems using ROS 2.

## Unified Robot Description Format (URDF)

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the physical and visual properties of a robot, including:

- Links (rigid parts of the robot)
- Joints (connections between links)
- Visual and collision properties
- Inertial properties
- Materials and colors

### URDF Structure

A basic URDF file contains:

```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

### Links and Joints

- **Links**: Represent rigid parts of the robot (e.g., chassis, arms, wheels)
- **Joints**: Define how links connect and move relative to each other
  - Fixed joints (no movement)
  - Revolute joints (rotational movement)
  - Continuous joints (unlimited rotation)
  - Prismatic joints (linear movement)

## Python Agent Integration

Python agents can interact with URDF-defined robots through ROS 2 interfaces. This allows for high-level control and AI-driven decision making.

### Robot State Publisher

The robot_state_publisher package takes the URDF and joint positions to publish the state of the robot's links and joints to tf2, which is used for coordinate transforms.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math

class RobotAgent(Node):
    def __init__(self):
        super().__init__('robot_agent')

        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Publisher for joint commands
        self.joint_publisher = self.create_publisher(
            JointState,
            'joint_commands',
            10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_state_callback(self, msg):
        # Process joint states and update agent's understanding of robot state
        self.get_logger().info(f'Received joint states: {msg.name}')

    def send_joint_commands(self, joint_positions):
        # Send commands to robot joints
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = joint_positions
        self.joint_publisher.publish(msg)
```

### Integration with AI Agents

Python-based AI agents can use URDF information to:
- Understand robot kinematics
- Plan movements within robot constraints
- Simulate robot behavior
- Interface with physics engines

## Practical Example: Controlling a URDF Robot

Here's an example of how a Python agent might control a URDF-defined robot:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class URDFAgent(Node):
    def __init__(self):
        super().__init__('urdf_agent')

        # Publisher for trajectory commands
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        # Timer for periodic control
        self.timer = self.create_timer(0.1, self.control_loop)

        self.time_step = 0

    def control_loop(self):
        # Generate a trajectory based on some logic
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3']

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [
            math.sin(self.time_step * 0.1),
            math.cos(self.time_step * 0.1),
            0.0
        ]

        # Set velocity constraints
        point.velocities = [
            0.1 * math.cos(self.time_step * 0.1),
            -0.1 * math.sin(self.time_step * 0.1),
            0.0
        ]

        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 seconds
        trajectory_msg.points = [point]

        self.trajectory_publisher.publish(trajectory_msg)
        self.time_step += 1

def main(args=None):
    rclpy.init(args=args)
    agent = URDFAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hardware Integration Notes

**RTX Workstation**: When simulating complex URDF robots with many joints, RTX-enabled workstations provide accelerated physics simulation through NVIDIA PhysX integration.

**Jetson Orin Nano**: For edge deployment, consider the computational requirements of processing URDF models and running Python agents simultaneously.

**RealSense Sensors**: When integrating with RealSense sensors, your Python agent can use URDF information to understand the robot's physical configuration and properly interpret sensor data in the robot's coordinate frame.

## Summary

URDF provides a standardized way to describe robot models, while Python agents can interface with these models through ROS 2. This combination enables sophisticated AI-driven control of robotic systems, bridging the gap between high-level decision making and low-level robot control.