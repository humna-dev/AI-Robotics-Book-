---
sidebar_position: 15
---

# Obstacle Navigation

This topic covers autonomous navigation in environments with obstacles, focusing on perception, path planning, and safe navigation strategies for humanoid robots.

## Obstacle Navigation Overview

Obstacle navigation is a fundamental capability for autonomous robots, enabling them to operate safely in complex environments. For humanoid robots, this includes both static and dynamic obstacles, as well as considerations for the robot's bipedal locomotion and human-like form factor.

### Key Components of Obstacle Navigation

1. **Perception**: Detecting and classifying obstacles in the environment
2. **Mapping**: Creating representations of free space and obstacles
3. **Path Planning**: Computing safe and efficient routes around obstacles
4. **Path Execution**: Following the planned path while avoiding collisions
5. **Recovery**: Handling navigation failures and replanning when needed

## Perception for Obstacle Detection

Humanoid robots typically use multiple sensors for comprehensive obstacle detection:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
from scipy.spatial import distance
from typing import List, Tuple

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Subscribers for different sensor types
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/points',
            self.pointcloud_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher for detected obstacles
        self.obstacle_pub = self.create_publisher(
            MarkerArray,
            '/detected_obstacles',
            10
        )

        # Publisher for obstacle points
        self.obstacle_points_pub = self.create_publisher(
            PointCloud2,
            '/obstacle_points',
            10
        )

        self.bridge = CvBridge()
        self.camera_info = None

        # Obstacle detection parameters
        self.min_obstacle_height = 0.1  # meters
        self.max_obstacle_height = 2.0  # meters
        self.min_distance = 0.3  # meters
        self.max_distance = 5.0  # meters

        self.get_logger().info('Obstacle Detector initialized')

    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        # Convert laser scan to points
        points = self.laser_to_points(msg)

        # Filter points based on height and distance
        filtered_points = self.filter_points(points)

        # Cluster points to identify individual obstacles
        obstacles = self.cluster_points(filtered_points)

        # Publish visualization markers
        self.publish_obstacle_markers(obstacles)

    def laser_to_points(self, scan_msg):
        """Convert laser scan to 3D points"""
        points = []
        angle = scan_msg.angle_min

        for i, range_val in enumerate(scan_msg.ranges):
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                z = 0.0  # Assume laser is at ground level for this example
                points.append([x, y, z])
            angle += scan_msg.angle_increment

        return np.array(points)

    def pointcloud_callback(self, msg):
        """Process point cloud data for 3D obstacle detection"""
        # Convert PointCloud2 to numpy array (simplified)
        # In practice, use point_cloud2.read_points_numpy or similar
        points = self.pointcloud_to_numpy(msg)

        # Filter points based on height and distance
        filtered_points = self.filter_points(points)

        # Remove ground plane
        non_ground_points = self.remove_ground_plane(filtered_points)

        # Cluster points to identify obstacles
        obstacles = self.cluster_points(non_ground_points)

        # Publish obstacle points
        self.publish_obstacle_points(non_ground_points)

    def camera_callback(self, msg):
        """Process camera image for visual obstacle detection"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform visual obstacle detection (simplified)
        # In practice, use deep learning models or traditional computer vision
        obstacle_masks = self.detect_obstacles_visual(cv_image)

        # Convert 2D image coordinates to 3D world coordinates
        if self.camera_info:
            obstacle_points = self.image_to_world_coordinates(obstacle_masks)

    def camera_info_callback(self, msg):
        """Store camera calibration information"""
        self.camera_info = msg

    def filter_points(self, points):
        """Filter points based on height and distance criteria"""
        if len(points) == 0:
            return points

        # Filter by distance
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        distance_mask = (distances >= self.min_distance) & (distances <= self.max_distance)

        # Filter by height
        height_mask = (points[:, 2] >= self.min_obstacle_height) & (points[:, 2] <= self.max_obstacle_height)

        return points[distance_mask & height_mask]

    def remove_ground_plane(self, points, distance_threshold=0.05):
        """Remove ground plane from point cloud using RANSAC"""
        if len(points) < 100:
            return points

        # Use Open3D for plane segmentation
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=1000
        )

        # Return points that are NOT part of the ground plane
        non_ground_points = np.asarray(pcd.points)[np.setdiff1d(np.arange(len(pcd.points)), inliers)]
        return non_ground_points

    def cluster_points(self, points, eps=0.3, min_samples=10):
        """Cluster points using DBSCAN to identify individual obstacles"""
        from sklearn.cluster import DBSCAN

        if len(points) < min_samples:
            return []

        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = clustering.labels_

        obstacles = []
        for label in set(labels):
            if label == -1:  # Noise points
                continue

            cluster_points = points[labels == label]
            centroid = np.mean(cluster_points, axis=0)
            size = np.std(cluster_points, axis=0)

            obstacles.append({
                'centroid': centroid,
                'size': size,
                'points': cluster_points,
                'label': label
            })

        return obstacles

    def detect_obstacles_visual(self, image):
        """Detect obstacles in camera image (simplified implementation)"""
        # In practice, use a trained deep learning model like YOLO or Mask R-CNN
        # For this example, we'll use simple color-based detection

        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for obstacle detection (example: red obstacles)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2
        return mask

    def image_to_world_coordinates(self, mask):
        """Convert 2D image coordinates to 3D world coordinates"""
        # Get points from mask
        y_coords, x_coords = np.where(mask > 0)

        if len(x_coords) == 0:
            return np.array([])

        # Use camera parameters to convert to 3D
        # This is a simplified approach - in practice, use more sophisticated methods
        if not self.camera_info:
            return np.array([])

        # For this example, we'll use a simple depth assumption
        # In practice, use stereo vision, depth sensor, or monocular depth estimation
        depth = 1.0  # Assume fixed depth for simplicity
        fx = self.camera_info.k[0]  # Focal length x
        fy = self.camera_info.k[4]  # Focal length y
        cx = self.camera_info.k[2]  # Principal point x
        cy = self.camera_info.k[5]  # Principal point y

        # Convert to 3D coordinates
        points_3d = []
        for x, y in zip(x_coords, y_coords):
            z = depth
            x_3d = (x - cx) * z / fx
            y_3d = (y - cy) * z / fy
            points_3d.append([x_3d, y_3d, z])

        return np.array(points_3d)

    def publish_obstacle_markers(self, obstacles):
        """Publish visualization markers for detected obstacles"""
        marker_array = MarkerArray()
        for i, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Set position
            marker.pose.position.x = obstacle['centroid'][0]
            marker.pose.position.y = obstacle['centroid'][1]
            marker.pose.position.z = obstacle['centroid'][2] + obstacle['size'][2] / 2  # Center at top of obstacle

            # Set dimensions
            marker.scale.x = max(0.2, obstacle['size'][0] * 2)  # Diameter
            marker.scale.y = max(0.2, obstacle['size'][1] * 2)  # Diameter
            marker.scale.z = obstacle['size'][2]  # Height

            # Set color (red for obstacles)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7  # Alpha

            marker_array.markers.append(marker)

        self.obstacle_pub.publish(marker_array)

    def publish_obstacle_points(self, points):
        """Publish obstacle points as PointCloud2"""
        # In practice, convert numpy array to PointCloud2 message
        # For this example, we'll just log the number of points
        self.get_logger().info(f'Published {len(points)} obstacle points')

    def pointcloud_to_numpy(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array (simplified)"""
        # This is a placeholder - in practice, use proper conversion
        # Use sensor_msgs.point_cloud2.read_points_numpy for actual conversion
        return np.array([[0, 0, 0]])  # Placeholder
```

## Local Navigation and Obstacle Avoidance

For real-time obstacle avoidance, humanoid robots need local navigation capabilities:

```python
import math
from enum import Enum

class NavigationState(Enum):
    IDLE = 0
    PLANNING = 1
    FOLLOWING = 2
    AVOIDING = 3
    RECOVERY = 4

class LocalNavigator(Node):
    def __init__(self):
        super().__init__('local_navigator')

        # Subscribers
        self.odom_sub = self.create_subscription(
            # Would use nav_msgs/Odometry in practice
            # Using Twist for this example
            Point,
            '/robot_position',
            self.odom_callback,
            10
        )

        self.obstacle_sub = self.create_subscription(
            MarkerArray,
            '/detected_obstacles',
            self.obstacle_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            Point,
            '/navigation_goal',
            self.goal_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.path_pub = self.create_publisher(
            # Would use nav_msgs/Path in practice
            Point,
            '/local_plan',
            10
        )

        # Navigation parameters
        self.robot_radius = 0.4  # meters
        self.max_speed = 0.5  # m/s
        self.max_turn_rate = 0.5  # rad/s
        self.min_distance_to_obstacle = 0.6  # meters
        self.arrival_threshold = 0.5  # meters

        # Navigation state
        self.navigation_state = NavigationState.IDLE
        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_goal = None
        self.obstacles = []
        self.local_path = []
        self.path_index = 0

        # Timer for navigation loop
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)  # 10 Hz

        self.get_logger().info('Local Navigator initialized')

    def odom_callback(self, msg):
        """Update robot position"""
        self.current_position = msg

    def obstacle_callback(self, msg):
        """Update obstacle information"""
        self.obstacles = []
        for marker in msg.markers:
            obstacle = {
                'position': (marker.pose.position.x, marker.pose.position.y),
                'radius': max(marker.scale.x, marker.scale.y) / 2.0
            }
            self.obstacles.append(obstacle)

    def goal_callback(self, msg):
        """Set new navigation goal"""
        self.current_goal = (msg.x, msg.y)
        self.navigation_state = NavigationState.PLANNING
        self.get_logger().info(f'New goal set: {self.current_goal}')

    def navigation_loop(self):
        """Main navigation control loop"""
        if self.navigation_state == NavigationState.IDLE:
            # Wait for goal
            pass
        elif self.navigation_state == NavigationState.PLANNING:
            # Plan path to goal
            self.plan_local_path()
        elif self.navigation_state == NavigationState.FOLLOWING:
            # Follow planned path
            self.follow_path()
        elif self.navigation_state == NavigationState.AVOIDING:
            # Avoid detected obstacles
            self.avoid_obstacles()
        elif self.navigation_state == NavigationState.RECOVERY:
            # Handle navigation failure
            self.recovery_behavior()

    def plan_local_path(self):
        """Plan a local path to the goal"""
        if not self.current_goal:
            return

        # Check if goal is reachable
        distance_to_goal = self.distance(
            (self.current_position.x, self.current_position.y),
            self.current_goal
        )

        if distance_to_goal < self.arrival_threshold:
            self.navigation_state = NavigationState.IDLE
            self.publish_cmd_vel(0.0, 0.0)  # Stop
            self.get_logger().info('Reached goal')
            return

        # Simple proportional navigation to goal
        goal_direction = self.normalize_vector(
            self.current_goal[0] - self.current_position.x,
            self.current_goal[1] - self.current_position.y
        )

        # Check for obstacles in the path
        if self.is_path_blocked(goal_direction):
            self.navigation_state = NavigationState.AVOIDING
            return

        # Set local path with goal direction
        self.local_path = [goal_direction]
        self.path_index = 0
        self.navigation_state = NavigationState.FOLLOWING

    def follow_path(self):
        """Follow the planned local path"""
        if not self.local_path or self.path_index >= len(self.local_path):
            self.navigation_state = NavigationState.PLANNING
            return

        # Get next path direction
        target_direction = self.local_path[self.path_index]

        # Check for obstacles in current direction
        if self.is_path_blocked(target_direction):
            self.navigation_state = NavigationState.AVOIDING
            return

        # Calculate velocity commands
        linear_vel = min(self.max_speed, self.distance_to_goal() * 0.5)
        angular_vel = math.atan2(target_direction[1], target_direction[0]) * 0.5

        # Apply limits
        angular_vel = max(-self.max_turn_rate, min(self.max_turn_rate, angular_vel))

        # Publish velocity commands
        self.publish_cmd_vel(linear_vel, angular_vel)

        # Check if we've reached the current path point
        if self.distance_to_path_point() < 0.3:
            self.path_index += 1
            if self.path_index >= len(self.local_path):
                self.navigation_state = NavigationState.PLANNING

    def avoid_obstacles(self):
        """Execute obstacle avoidance behavior"""
        # Simple obstacle avoidance using potential fields
        avoidance_vector = self.calculate_avoidance_vector()

        if avoidance_vector is not None:
            # Calculate avoidance velocity
            linear_vel = min(self.max_speed * 0.5, 0.2)  # Slower during avoidance
            angular_vel = math.atan2(avoidance_vector[1], avoidance_vector[0]) * 1.0

            # Apply limits
            angular_vel = max(-self.max_turn_rate, min(self.max_turn_rate, angular_vel))

            # Publish velocity commands
            self.publish_cmd_vel(linear_vel, angular_vel)
        else:
            # No clear direction, stop and replan
            self.publish_cmd_vel(0.0, 0.0)
            self.navigation_state = NavigationState.PLANNING

    def calculate_avoidance_vector(self):
        """Calculate direction to avoid obstacles"""
        current_pos = (self.current_position.x, self.current_position.y)

        # Calculate repulsive forces from obstacles
        repulsive_force = [0.0, 0.0]

        for obstacle in self.obstacles:
            obs_pos = obstacle['position']
            distance = self.distance(current_pos, obs_pos)

            if distance < self.min_distance_to_obstacle:
                # Calculate repulsive force direction (away from obstacle)
                dx = current_pos[0] - obs_pos[0]
                dy = current_pos[1] - obs_pos[1]

                # Normalize and scale by inverse distance squared
                length = math.sqrt(dx*dx + dy*dy)
                if length > 0:
                    dx /= length
                    dy /= length

                    # Scale by inverse square of distance (closer = stronger repulsion)
                    scale = (self.min_distance_to_obstacle - distance) / self.min_distance_to_obstacle
                    scale = scale * scale * 10.0  # Amplify the effect

                    repulsive_force[0] += dx * scale
                    repulsive_force[1] += dy * scale

        # If there's a clear goal direction, add attractive force
        if self.current_goal:
            goal_direction = self.normalize_vector(
                self.current_goal[0] - current_pos[0],
                self.current_goal[1] - current_pos[1]
            )

            # Scale attractive force
            attractive_force = [goal_direction[0] * 2.0, goal_direction[1] * 2.0]

            # Combine forces
            combined_force = [
                attractive_force[0] + repulsive_force[0],
                attractive_force[1] + repulsive_force[1]
            ]
        else:
            combined_force = repulsive_force

        # Normalize the combined force
        length = math.sqrt(combined_force[0]**2 + combined_force[1]**2)
        if length > 0:
            return [combined_force[0] / length, combined_force[1] / length]

        return None

    def is_path_blocked(self, direction):
        """Check if the path in the given direction is blocked by obstacles"""
        current_pos = (self.current_position.x, self.current_position.y)

        # Check a few points ahead in the direction
        for distance in [0.5, 1.0, 1.5]:  # Check at 0.5m, 1.0m, 1.5m ahead
            check_pos = (
                current_pos[0] + direction[0] * distance,
                current_pos[1] + direction[1] * distance
            )

            for obstacle in self.obstacles:
                obs_pos = obstacle['position']
                obs_radius = obstacle['radius']

                distance_to_obstacle = self.distance(check_pos, obs_pos)
                if distance_to_obstacle < (obs_radius + self.robot_radius):
                    return True  # Path is blocked

        return False

    def distance_to_goal(self):
        """Calculate distance to the current goal"""
        if not self.current_goal:
            return float('inf')

        return self.distance(
            (self.current_position.x, self.current_position.y),
            self.current_goal
        )

    def distance_to_path_point(self):
        """Calculate distance to the current path point"""
        if not self.local_path or self.path_index >= len(self.local_path):
            return float('inf')

        # Simplified: just return distance to goal
        return self.distance_to_goal()

    def distance(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def normalize_vector(self, x, y):
        """Normalize a 2D vector"""
        length = math.sqrt(x*x + y*y)
        if length > 0:
            return [x / length, y / length]
        return [0.0, 0.0]

    def publish_cmd_vel(self, linear, angular):
        """Publish velocity commands"""
        cmd_msg = Twist()
        cmd_msg.linear.x = linear
        cmd_msg.angular.z = angular
        self.cmd_vel_pub.publish(cmd_msg)

    def recovery_behavior(self):
        """Handle navigation recovery when stuck"""
        # Stop the robot
        self.publish_cmd_vel(0.0, 0.0)

        # Log the recovery attempt
        self.get_logger().warn('Navigation recovery: Robot appears to be stuck')

        # After a delay, try to replan
        # In practice, use a timer or action client
        self.navigation_state = NavigationState.PLANNING
```

## Humanoid-Specific Navigation Considerations

Humanoid robots have unique navigation requirements due to their form factor and locomotion method:

```python
class HumanoidNavigator(LocalNavigator):
    def __init__(self):
        super().__init__()

        # Humanoid-specific parameters
        self.step_size = 0.3  # Maximum step size for bipedal locomotion
        self.turn_radius = 0.4  # Minimum turning radius
        self.zmp_stability_margin = 0.05  # Zero Moment Point safety margin
        self.com_height = 0.8  # Center of mass height

        # Walking state machine
        self.walking_state = 'standing'
        self.support_foot = 'left'  # Which foot is supporting weight
        self.swing_foot_trajectory = []  # Planned trajectory for swing foot

        # Subscribe to additional humanoid-specific topics
        self.imu_sub = self.create_subscription(
            # Would use sensor_msgs/Imu
            Point,
            '/imu_filtered',
            self.imu_callback,
            10
        )

        self.foot_pressure_sub = self.create_subscription(
            # Would use custom message for foot pressure sensors
            Point,
            '/foot_pressure',
            self.foot_pressure_callback,
            10
        )

        self.get_logger().info('Humanoid Navigator initialized')

    def imu_callback(self, msg):
        """Handle IMU data for balance-aware navigation"""
        # Extract orientation and angular velocity
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.x, msg.y, msg.z, msg.w  # Assuming the Point message contains quaternion
        )

        # Check if robot is stable enough for navigation
        if abs(roll) > 0.3 or abs(pitch) > 0.3:  # 17 degree threshold
            self.get_logger().warn('Robot is unstable, stopping navigation')
            self.publish_cmd_vel(0.0, 0.0)
            return

        # Adjust navigation behavior based on balance
        self.adjust_navigation_for_balance(roll, pitch)

    def foot_pressure_callback(self, msg):
        """Handle foot pressure sensor data"""
        # Check if robot is properly supporting weight
        left_pressure = msg.x  # Assuming Point contains pressure data
        right_pressure = msg.y

        # Ensure stable weight transfer during walking
        if self.walking_state == 'stepping':
            if self.support_foot == 'left' and left_pressure < 0.1:
                self.get_logger().warn('Unexpected weight transfer detected')
            elif self.support_foot == 'right' and right_pressure < 0.1:
                self.get_logger().warn('Unexpected weight transfer detected')

    def adjust_navigation_for_balance(self, roll, pitch):
        """Adjust navigation commands to maintain balance"""
        # Reduce speed when the robot is tilted
        balance_factor = max(0.1, 1.0 - abs(roll) - abs(pitch))

        # In practice, this would modify the velocity commands
        # For now, just log the adjustment
        self.get_logger().info(f'Balance adjustment factor: {balance_factor:.2f}')

    def plan_bipedal_path(self, goal):
        """Plan a path considering bipedal locomotion constraints"""
        # Check if direct path is feasible considering step size
        direct_path = self.calculate_direct_path(goal)

        if self.is_path_feasible_bipedal(direct_path):
            return direct_path

        # If direct path is not feasible, plan around obstacles
        # considering step-by-step movement
        return self.plan_bipedal_around_obstacles(goal)

    def is_path_feasible_bipedal(self, path):
        """Check if a path is feasible for bipedal locomotion"""
        # Check if each step in the path is within step size limits
        for i in range(1, len(path)):
            step_distance = self.distance(path[i-1], path[i])
            if step_distance > self.step_size * 1.5:  # Allow 50% over step size
                return False

        return True

    def calculate_direct_path(self, goal):
        """Calculate a direct path to the goal"""
        current_pos = (self.current_position.x, self.current_position.y)
        return [current_pos, goal]

    def plan_bipedal_around_obstacles(self, goal):
        """Plan a path around obstacles considering bipedal constraints"""
        # This is a simplified implementation
        # In practice, use a more sophisticated path planner that considers
        # step size, turning radius, and balance constraints

        current_pos = (self.current_position.x, self.current_position.y)

        # Generate waypoints that respect step size
        dx = goal[0] - current_pos[0]
        dy = goal[1] - current_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate number of steps needed
        num_steps = int(distance / self.step_size) + 1

        path = []
        for i in range(num_steps + 1):
            progress = i / num_steps if num_steps > 0 else 0
            x = current_pos[0] + dx * progress
            y = current_pos[1] + dy * progress
            path.append((x, y))

        return path

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
```

## Dynamic Obstacle Handling

Humanoid robots must handle moving obstacles, especially in human-populated environments:

```python
from collections import deque

class DynamicObstacleHandler(HumanoidNavigator):
    def __init__(self):
        super().__init__()

        # Track moving obstacles
        self.moving_obstacles = {}  # ID -> trajectory history
        self.obstacle_velocities = {}  # ID -> estimated velocity
        self.prediction_horizon = 2.0  # seconds to predict into the future

        # Timer for dynamic obstacle processing
        self.dynamic_timer = self.create_timer(0.5, self.process_dynamic_obstacles)

    def obstacle_callback(self, msg):
        """Enhanced obstacle callback that tracks moving obstacles"""
        super().obstacle_callback(msg)

        # Get current timestamp
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Process each obstacle to see if it's moving
        for i, marker in enumerate(msg.markers):
            obstacle_id = marker.id
            position = (marker.pose.position.x, marker.pose.position.y)

            if obstacle_id in self.moving_obstacles:
                # Update existing obstacle track
                self.moving_obstacles[obstacle_id].append({
                    'position': position,
                    'timestamp': current_time
                })

                # Keep only recent history (last 5 seconds)
                self.moving_obstacles[obstacle_id] = [
                    obs for obs in self.moving_obstacles[obstacle_id]
                    if current_time - obs['timestamp'] <= 5.0
                ]

                # Estimate velocity if we have enough history
                if len(self.moving_obstacles[obstacle_id]) >= 2:
                    self.estimate_obstacle_velocity(obstacle_id)
            else:
                # New obstacle - create track
                self.moving_obstacles[obstacle_id] = deque(maxlen=10)  # Keep last 10 positions
                self.moving_obstacles[obstacle_id].append({
                    'position': position,
                    'timestamp': current_time
                })

    def estimate_obstacle_velocity(self, obstacle_id):
        """Estimate the velocity of a moving obstacle"""
        history = list(self.moving_obstacles[obstacle_id])
        if len(history) < 2:
            return

        # Use the most recent and oldest positions to estimate velocity
        recent = history[-1]
        old = history[0]

        dt = recent['timestamp'] - old['timestamp']
        if dt <= 0:
            return

        dx = recent['position'][0] - old['position'][0]
        dy = recent['position'][1] - old['position'][1]

        vx = dx / dt
        vy = dy / dt

        self.obstacle_velocities[obstacle_id] = (vx, vy)

        self.get_logger().info(f'Estimated velocity for obstacle {obstacle_id}: ({vx:.2f}, {vy:.2f}) m/s')

    def process_dynamic_obstacles(self):
        """Process dynamic obstacles and adjust navigation"""
        for obstacle_id, velocity in self.obstacle_velocities.items():
            if obstacle_id in self.moving_obstacles:
                current_pos = self.moving_obstacles[obstacle_id][-1]['position']

                # Predict future positions
                predicted_positions = self.predict_obstacle_trajectory(
                    current_pos, velocity, self.prediction_horizon
                )

                # Check if predicted path intersects with our navigation path
                if self.would_collide_with_predicted_obstacle(predicted_positions):
                    self.get_logger().info(f'Avoiding predicted path of moving obstacle {obstacle_id}')
                    # Trigger avoidance behavior
                    self.navigation_state = NavigationState.AVOIDING

    def predict_obstacle_trajectory(self, current_pos, velocity, horizon, steps=10):
        """Predict future positions of a moving obstacle"""
        dt = horizon / steps
        predicted_positions = []

        for i in range(steps + 1):
            t = i * dt
            predicted_x = current_pos[0] + velocity[0] * t
            predicted_y = current_pos[1] + velocity[1] * t
            predicted_positions.append((predicted_x, predicted_y))

        return predicted_positions

    def would_collide_with_predicted_obstacle(self, predicted_positions):
        """Check if our planned path would collide with predicted obstacle positions"""
        # Check if any predicted obstacle position is close to our planned path
        for obs_pos in predicted_positions:
            # Simple check: if obstacle will be within robot's safety radius
            distance = self.distance(
                (self.current_position.x, self.current_position.y),
                obs_pos
            )

            if distance < (self.robot_radius + 0.3):  # 0.3m safety buffer
                return True

        return False

    def avoid_moving_obstacles(self):
        """Enhanced obstacle avoidance for moving obstacles"""
        # Calculate avoidance considering obstacle velocities
        avoidance_vector = self.calculate_moving_obstacle_avoidance()

        if avoidance_vector is not None:
            # Use the avoidance vector for navigation
            linear_vel = min(self.max_speed * 0.7, 0.3)  # Slightly faster than static avoidance
            angular_vel = math.atan2(avoidance_vector[1], avoidance_vector[0]) * 1.2

            # Apply limits
            angular_vel = max(-self.max_turn_rate, min(self.max_turn_rate, angular_vel))

            # Publish velocity commands
            self.publish_cmd_vel(linear_vel, angular_vel)
        else:
            # No safe direction found, stop and wait
            self.publish_cmd_vel(0.0, 0.0)
            self.get_logger().warn('No safe direction found, waiting for clear path')
```

## Hardware Integration Notes

**RTX Workstation**: Real-time obstacle detection and navigation planning benefit from GPU acceleration, especially for processing 3D point clouds and running deep learning models.

**Jetson Orin Nano**: For humanoid robots, edge computing platforms provide the necessary processing power for perception and navigation while maintaining low latency.

**RealSense Integration**: 3D cameras like RealSense provide rich depth information crucial for accurate obstacle detection and mapping.

## Safety Considerations

Obstacle navigation for humanoid robots must prioritize safety:

1. **Emergency Stop**: Implement immediate stop capability when obstacles are too close
2. **Safe Velocities**: Limit speeds based on obstacle proximity and robot stability
3. **Redundant Perception**: Use multiple sensor modalities for robust obstacle detection
4. **Human Awareness**: Special consideration for navigating around humans

## Summary

Obstacle navigation for humanoid robots requires sophisticated perception, planning, and control systems that account for the unique challenges of bipedal locomotion. By combining multiple sensor modalities, predicting dynamic obstacles, and maintaining balance during navigation, humanoid robots can safely operate in complex environments. The integration of real-time obstacle detection with stable locomotion patterns enables humanoid robots to navigate effectively while maintaining safety.