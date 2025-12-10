---
sidebar_position: 14
---

# Simulated Humanoid

This topic covers the design, modeling, and simulation of humanoid robots, focusing on creating realistic humanoid models and implementing coordinated control systems.

## Humanoid Robot Design

Humanoid robots are designed to mimic human form and behavior, typically featuring a head, torso, two arms, and two legs. The design process involves:

1. **Kinematic Structure**: Defining the joint configuration and degrees of freedom
2. **Dynamic Properties**: Specifying mass, inertia, and actuator characteristics
3. **Actuation System**: Selecting motors, servos, and control electronics
4. **Sensing System**: Integrating cameras, IMUs, force sensors, and other sensors
5. **Control Architecture**: Designing the software and hardware control systems

### URDF Model for Humanoid Robot

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.5"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Arm (similar to left) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Leg (similar to left) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.05 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Sensors -->
  <gazebo reference="head">
    <sensor type="camera" name="head_camera">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <visualize>true</visualize>
    </sensor>
  </gazebo>

  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_humanoid</robotNamespace>
    </plugin>
  </gazebo>
</robot>
```

## Humanoid Control Systems

Humanoid robots require sophisticated control systems to maintain balance and execute coordinated movements:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publisher for joint trajectories
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Subscriber for IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # Initialize joint positions
        self.current_joint_positions = {}
        self.imu_data = None
        self.cmd_vel = Twist()

        # Balance control parameters
        self.balance_kp = 10.0
        self.balance_kd = 1.0

        self.get_logger().info('Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Update IMU data for balance control"""
        self.imu_data = msg

    def cmd_vel_callback(self, msg):
        """Update velocity command"""
        self.cmd_vel = msg

    def control_loop(self):
        """Main control loop for humanoid robot"""
        # Check if we have required sensor data
        if not self.imu_data:
            return

        # Calculate desired joint positions based on balance and movement goals
        desired_positions = self.calculate_desired_positions()

        # Publish joint trajectory
        self.publish_joint_trajectory(desired_positions)

    def calculate_desired_positions(self):
        """Calculate desired joint positions for balance and movement"""
        desired_positions = {}

        # Get current orientation from IMU
        roll, pitch, yaw = self.quaternion_to_euler(
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w
        )

        # Balance control: correct for pitch and roll
        pitch_correction = -self.balance_kp * pitch - self.balance_kd * self.imu_data.angular_velocity.y
        roll_correction = -self.balance_kp * roll - self.balance_kd * self.imu_data.angular_velocity.x

        # Apply corrections to hip and ankle joints
        desired_positions['left_hip_joint'] = pitch_correction * 0.1
        desired_positions['right_hip_joint'] = pitch_correction * 0.1
        desired_positions['left_ankle_joint'] = -pitch_correction * 0.05
        desired_positions['right_ankle_joint'] = -pitch_correction * 0.05

        # Apply roll corrections to ankle joints
        desired_positions['left_ankle_joint'] += roll_correction * 0.05
        desired_positions['right_ankle_joint'] -= roll_correction * 0.05

        # Movement control based on cmd_vel
        if abs(self.cmd_vel.linear.x) > 0.01:
            # Walking motion - simplified
            phase = self.get_clock().now().nanoseconds / 1e9 * 2.0  # 2 Hz walking
            walk_amplitude = 0.1 * abs(self.cmd_vel.linear.x)

            desired_positions['left_hip_joint'] += math.sin(phase) * walk_amplitude
            desired_positions['right_hip_joint'] += math.sin(phase + math.pi) * walk_amplitude
            desired_positions['left_knee_joint'] = abs(math.sin(phase)) * walk_amplitude * 0.5
            desired_positions['right_knee_joint'] = abs(math.sin(phase + math.pi)) * walk_amplitude * 0.5

        # Arm positioning for balance
        desired_positions['left_shoulder_joint'] = -roll_correction * 0.5
        desired_positions['right_shoulder_joint'] = roll_correction * 0.5

        # Default positions for joints not controlled by balance
        default_positions = {
            'neck_joint': 0.0,
            'left_elbow_joint': -0.5,
            'right_elbow_joint': -0.5,
        }

        # Combine calculated and default positions
        for joint, pos in default_positions.items():
            if joint not in desired_positions:
                desired_positions[joint] = pos

        return desired_positions

    def publish_joint_trajectory(self, desired_positions):
        """Publish joint trajectory message"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = list(desired_positions.keys())

        point = JointTrajectoryPoint()
        point.positions = list(desired_positions.values())
        point.velocities = [0.0] * len(desired_positions)  # Zero velocity for simplicity
        point.accelerations = [0.0] * len(desired_positions)  # Zero acceleration for simplicity
        point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms

        trajectory_msg.points = [point]
        self.joint_trajectory_pub.publish(trajectory_msg)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def get_joint_names(self):
        """Get list of all joint names"""
        return [
            'neck_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]
```

## Walking Pattern Generation

Generating stable walking patterns is a key challenge in humanoid robotics:

```python
class WalkingPatternGenerator:
    def __init__(self):
        self.step_length = 0.3  # meters
        self.step_height = 0.05  # meters
        self.step_duration = 1.0  # seconds
        self.zmp_margin = 0.05  # Zero Moment Point safety margin

    def generate_walk_trajectory(self, steps, direction='forward'):
        """Generate a walking trajectory for a given number of steps"""
        trajectory = []

        for step in range(steps):
            # Calculate phase of walking cycle
            phase = (step % 2)  # 0 for left foot, 1 for right foot

            # Generate trajectory for single step
            step_trajectory = self.generate_single_step(phase, direction)
            trajectory.extend(step_trajectory)

        return trajectory

    def generate_single_step(self, phase, direction):
        """Generate trajectory for a single step"""
        step_trajectory = []

        # Number of points per step
        points_per_step = 20
        dt = self.step_duration / points_per_step

        for i in range(points_per_step):
            t = i * dt
            progress = t / self.step_duration  # 0 to 1

            # Calculate foot position
            if direction == 'forward':
                x_offset = progress * self.step_length
            elif direction == 'backward':
                x_offset = -progress * self.step_length
            else:
                x_offset = 0.0

            # Foot height profile (sinusoidal lift and place)
            if progress < 0.3:  # Lift phase
                height = self.step_height * math.sin(progress / 0.3 * math.pi / 2)
            elif progress > 0.7:  # Place phase
                height = self.step_height * math.sin((1 - progress) / 0.3 * math.pi / 2)
            else:  # Constant height phase
                height = self.step_height

            # Calculate body position to maintain balance
            body_x = x_offset / 2  # Simplified: body moves half the foot distance
            body_z = 0.8  # Maintain constant height

            step_trajectory.append({
                'time': t,
                'foot_position': (x_offset, 0, height),
                'body_position': (body_x, 0, body_z),
                'support_foot': 'right' if phase == 0 else 'left'
            })

        return step_trajectory

    def calculate_joint_angles_for_pose(self, body_pose, foot_poses):
        """Calculate joint angles to achieve a desired body and foot pose"""
        # This is a simplified inverse kinematics example
        # In practice, this would use more sophisticated IK algorithms

        # Calculate hip joint angles to position feet
        left_hip_angle = math.atan2(foot_poses['left'][1], foot_poses['left'][0])
        right_hip_angle = math.atan2(foot_poses['right'][1], foot_poses['right'][0])

        # Calculate knee angles to achieve foot height
        leg_length = 0.8  # Simplified leg length
        left_knee_angle = math.acos((leg_length**2 + leg_length**2 - foot_poses['left'][2]**2) / (2 * leg_length**2))
        right_knee_angle = math.acos((leg_length**2 + leg_length**2 - foot_poses['right'][2]**2) / (2 * leg_length**2))

        return {
            'left_hip_joint': left_hip_angle,
            'right_hip_joint': right_hip_angle,
            'left_knee_joint': left_knee_angle,
            'right_knee_joint': right_knee_angle
        }
```

## Simulation Integration

Integrating the humanoid controller with simulation environments:

```python
class HumanoidSimulationBridge(HumanoidController):
    def __init__(self):
        super().__init__()

        # Publisher for Gazebo model states
        self.model_state_pub = self.create_publisher(
            # Would use gazebo_msgs/ModelState in practice
            # For this example, we'll use a generic message
            JointTrajectory,
            '/gazebo/set_model_state',
            10
        )

        # Timer for simulation synchronization
        self.sim_timer = self.create_timer(0.01, self.sim_sync_loop)  # 100 Hz

    def sim_sync_loop(self):
        """Synchronization loop with simulation"""
        # In a real implementation, this would interface with Gazebo
        # to ensure proper timing and state synchronization
        pass

    def publish_to_simulation(self, joint_commands):
        """Publish commands to the simulation environment"""
        # Create and publish appropriate messages for the simulation
        # This would typically involve Gazebo plugins or other simulation interfaces
        pass
```

## Hardware Integration Notes

**RTX Workstation**: Humanoid robot simulation with realistic physics and control systems requires significant computational resources.

**Jetson Orin Nano**: For physical humanoid robots, edge computing platforms provide the necessary processing power for real-time control and perception.

**RealSense Integration**: 3D perception is crucial for humanoid robots to understand their environment and navigate safely.

## Challenges in Humanoid Robotics

Humanoid robotics faces several unique challenges:

1. **Balance Control**: Maintaining stability with a high center of gravity
2. **Real-time Control**: Processing sensor data and computing control commands at high frequency
3. **Complex Kinematics**: Managing many degrees of freedom for coordinated movement
4. **Power Management**: Efficiently using power for multiple actuators
5. **Safety**: Ensuring safe operation around humans

## Summary

Simulated humanoid robots provide a safe and cost-effective way to develop and test humanoid control algorithms before deployment on physical robots. Creating realistic models and implementing stable control systems are key challenges in humanoid robotics. The integration of perception, planning, and control systems enables humanoid robots to perform complex tasks while maintaining balance and safety.