---
sidebar_position: 2
---

# ROS 2 Nodes & Topics

This topic covers the fundamental concepts of ROS 2 nodes and topics, which form the backbone of robotic communication systems.

## Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node can publish or subscribe to topics, provide or use services, and own or interact with action servers.

### Creating a Node

To create a ROS 2 node, you typically:

1. Initialize the ROS 2 client library
2. Create a node instance
3. Define publishers, subscribers, services, or actions
4. Spin the node to process callbacks
5. Clean up resources

### Node Best Practices

- Keep nodes focused on a single responsibility
- Use meaningful names for nodes
- Handle errors gracefully
- Implement proper lifecycle management

## Topics

Topics are named buses over which nodes exchange messages. Publishers send messages to a topic, and subscribers receive messages from a topic. This creates a many-to-many relationship between publishers and subscribers.

### Topic Communication

- **Publishers** send data to a topic
- **Subscribers** receive data from a topic
- Communication is asynchronous
- Multiple publishers and subscribers can exist for the same topic

### Quality of Service (QoS)

ROS 2 provides Quality of Service settings that allow you to configure the behavior of topic communication, including:
- Reliability (reliable vs best effort)
- Durability (transient local vs volatile)
- History (keep last vs keep all)

## Practical Implementation

Here's a simple example of a publisher and subscriber in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_publisher)
    rclpy.spin(minimal_subscriber)

    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hardware Considerations

**RealSense Integration**: When working with RealSense sensors, you'll often create nodes that publish sensor data to topics for processing by other nodes.

**Latency Management**: For real-time robotics applications, consider the latency implications of topic communication, especially when using cloud-based processing.

## Summary

Nodes and topics form the foundation of ROS 2 communication. Understanding these concepts is crucial for building distributed robotic systems that can coordinate complex behaviors through modular software components.