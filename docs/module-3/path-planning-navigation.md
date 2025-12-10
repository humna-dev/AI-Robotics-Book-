---
sidebar_position: 9
---

# Path Planning & Navigation

This topic covers path planning and navigation algorithms in robotics, focusing on how to plan and execute safe, efficient paths for mobile robots in complex environments.

## Path Planning Overview

Path planning is the process of determining a safe and optimal route from a starting position to a goal position. It's a fundamental capability for mobile robots and autonomous systems.

### Types of Path Planning

- **Global Path Planning**: Plan a route based on a known map
- **Local Path Planning**: Adjust the path in real-time based on sensor data
- **Motion Planning**: Consider robot dynamics and kinematics
- **Multi-objective Planning**: Balance multiple criteria (distance, safety, energy)

## Navigation Stack

The navigation stack typically includes:

1. **Localization**: Determine robot position in the environment
2. **Mapping**: Create or update environment representation
3. **Path Planning**: Compute optimal route
4. **Path Execution**: Control robot to follow the path
5. **Recovery**: Handle navigation failures

### ROS 2 Navigation (Nav2)

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

    def send_goal(self, x, y, theta):
        # Wait for action server
        self.action_client.wait_for_server()

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Convert theta to quaternion
        import math
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current pose: {feedback.current_pose}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

## Common Path Planning Algorithms

### A* Algorithm

A* is a popular graph-based path planning algorithm that uses heuristics to find optimal paths efficiently:

```python
import heapq
import numpy as np

def a_star(grid, start, goal):
    """
    A* path planning algorithm
    grid: 2D array where 0 = free space, 1 = obstacle
    start, goal: (row, col) tuples
    """
    rows, cols = grid.shape
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for neighbor in get_neighbors(current, rows, cols):
            if grid[neighbor] == 1:  # Obstacle
                continue

            tentative_g = g_score[current] + distance(current, neighbor)

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # No path found

def heuristic(a, b):
    """Manhattan distance heuristic"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(pos, rows, cols):
    """Get valid neighboring positions"""
    neighbors = []
    for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:  # 4-connected
        r, c = pos[0] + dr, pos[1] + dc
        if 0 <= r < rows and 0 <= c < cols:
            neighbors.append((r, c))
    return neighbors

def distance(a, b):
    """Euclidean distance"""
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
```

### Dijkstra's Algorithm

Dijkstra's algorithm finds the shortest path in a weighted graph without using heuristics:

```python
import heapq

def dijkstra(graph, start, end):
    """
    Dijkstra's algorithm for shortest path
    graph: adjacency list with weights
    """
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    previous = {}
    pq = [(0, start)]

    while pq:
        current_distance, current = heapq.heappop(pq)

        if current == end:
            break

        if current_distance > distances[current]:
            continue

        for neighbor, weight in graph[current].items():
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current
                heapq.heappush(pq, (distance, neighbor))

    # Reconstruct path
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = previous.get(current)
    path.reverse()

    return path if path[0] == start else []
```

### RRT (Rapidly-exploring Random Trees)

RRT is useful for high-dimensional configuration spaces:

```python
import numpy as np
import random

class RRT:
    def __init__(self, start, goal, bounds, obstacles):
        self.start = start
        self.goal = goal
        self.bounds = bounds  # (min_x, max_x, min_y, max_y)
        self.obstacles = obstacles
        self.nodes = [start]
        self.edges = []
        self.goal_threshold = 1.0

    def plan(self, max_iterations=1000):
        for i in range(max_iterations):
            # Sample random point
            rand_point = self.sample_random_point()

            # Find nearest node
            nearest_idx = self.nearest_node(rand_point)
            nearest_node = self.nodes[nearest_idx]

            # Extend towards random point
            new_point = self.extend_towards(nearest_node, rand_point)

            # Check collision
            if not self.check_collision(nearest_node, new_point):
                self.nodes.append(new_point)
                self.edges.append((nearest_idx, len(self.nodes) - 1))

                # Check if goal is reached
                if self.distance(new_point, self.goal) < self.goal_threshold:
                    return self.reconstruct_path(len(self.nodes) - 1)

        return []  # No path found

    def sample_random_point(self):
        if random.random() < 0.05:  # 5% chance to sample goal
            return self.goal
        return (
            random.uniform(self.bounds[0], self.bounds[1]),
            random.uniform(self.bounds[2], self.bounds[3])
        )

    def nearest_node(self, point):
        min_dist = float('inf')
        nearest_idx = 0
        for i, node in enumerate(self.nodes):
            dist = self.distance(node, point)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        return nearest_idx

    def extend_towards(self, from_node, to_node, step_size=1.0):
        dist = self.distance(from_node, to_node)
        if dist <= step_size:
            return to_node
        ratio = step_size / dist
        new_x = from_node[0] + ratio * (to_node[0] - from_node[0])
        new_y = from_node[1] + ratio * (to_node[1] - from_node[1])
        return (new_x, new_y)

    def distance(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def check_collision(self, p1, p2):
        # Simple collision checking - in practice, use more sophisticated methods
        for obstacle in self.obstacles:
            if self.line_intersects_obstacle(p1, p2, obstacle):
                return True
        return False

    def line_intersects_obstacle(self, p1, p2, obstacle):
        # Simplified collision detection
        # In practice, use proper line-segment intersection algorithms
        return False

    def reconstruct_path(self, goal_idx):
        path = [self.goal]
        current_idx = goal_idx
        while current_idx != 0:
            for edge in self.edges:
                if edge[1] == current_idx:
                    current_idx = edge[0]
                    path.append(self.nodes[current_idx])
                    break
        path.reverse()
        return path
```

## Local Navigation and Obstacle Avoidance

Local navigation handles dynamic obstacles and fine adjustments to the global plan:

```python
import math

class LocalNavigator:
    def __init__(self, robot_radius=0.3, max_speed=1.0, max_turn_rate=1.0):
        self.robot_radius = robot_radius
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate
        self.current_goal = None
        self.path = []
        self.path_index = 0

    def update_local_plan(self, current_pose, sensor_data, target_goal):
        # Check if we need to replan
        if self.should_replan(current_pose, sensor_data):
            return self.avoid_obstacles(current_pose, sensor_data, target_goal)
        else:
            return self.follow_global_path(current_pose, target_goal)

    def should_replan(self, current_pose, sensor_data):
        # Check if there are unexpected obstacles on the path
        for obstacle in sensor_data:
            if self.distance(current_pose, obstacle) < self.robot_radius * 2:
                return True
        return False

    def avoid_obstacles(self, current_pose, sensor_data, target_goal):
        # Simple obstacle avoidance using sensor data
        # In practice, use algorithms like Dynamic Window Approach (DWA)

        # Calculate repulsive forces from obstacles
        repulsive_force = [0, 0]
        for obstacle in sensor_data:
            dist = self.distance(current_pose, obstacle)
            if dist < 2.0:  # Influence radius
                force_magnitude = 1.0 / (dist * dist) if dist > 0.1 else 100
                direction = [
                    current_pose[0] - obstacle[0],
                    current_pose[1] - obstacle[1]
                ]
                # Normalize
                length = math.sqrt(direction[0]**2 + direction[1]**2)
                if length > 0:
                    direction[0] /= length
                    direction[1] /= length

                repulsive_force[0] += direction[0] * force_magnitude
                repulsive_force[1] += direction[1] * force_magnitude

        # Calculate attractive force to goal
        goal_direction = [
            target_goal[0] - current_pose[0],
            target_goal[1] - current_pose[1]
        ]
        distance_to_goal = math.sqrt(goal_direction[0]**2 + goal_direction[1]**2)
        if distance_to_goal > 0:
            goal_direction[0] /= distance_to_goal
            goal_direction[1] /= distance_to_goal

        # Combine forces
        combined_force = [
            goal_direction[0] + repulsive_force[0],
            goal_direction[1] + repulsive_force[1]
        ]

        # Normalize and scale
        force_length = math.sqrt(combined_force[0]**2 + combined_force[1]**2)
        if force_length > 0:
            combined_force[0] /= force_length
            combined_force[1] /= force_length

        return combined_force

    def follow_global_path(self, current_pose, target_goal):
        # Follow the global path with simple proportional control
        target_direction = [
            target_goal[0] - current_pose[0],
            target_goal[1] - current_pose[1]
        ]

        distance = math.sqrt(target_direction[0]**2 + target_direction[1]**2)
        if distance > 0:
            target_direction[0] /= distance
            target_direction[1] /= distance

        return target_direction

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
```

## Hardware Integration Notes

**RTX Workstation**: Path planning algorithms can benefit from GPU acceleration, especially for complex algorithms like RRT* or sampling-based planners.

**Jetson Orin Nano**: For edge deployment, ensure path planning algorithms are optimized for the computational constraints of the platform.

**RealSense Integration**: Use depth data from RealSense sensors for local obstacle detection and avoidance.

## Summary

Path planning and navigation are critical capabilities for mobile robots. By combining global planning with local obstacle avoidance, robots can navigate complex environments safely and efficiently. The choice of algorithm depends on the specific requirements of the application, including environment complexity, real-time constraints, and accuracy requirements.