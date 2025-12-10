---
sidebar_position: 11
---

# Voice-to-Action

This topic covers voice command processing and conversion to robotic actions, enabling natural language interaction with robotic systems.

## Voice-to-Action Overview

Voice-to-Action systems enable robots to understand and execute commands given in natural language. This involves several key components:

1. **Speech Recognition**: Converting audio to text
2. **Natural Language Understanding**: Interpreting the meaning of commands
3. **Action Mapping**: Translating understood commands to robotic actions
4. **Execution**: Carrying out the requested actions

## Architecture of Voice-to-Action Systems

### Speech Recognition Pipeline

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import speech_recognition as sr
import threading

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')

        # Publisher for recognized text
        self.text_publisher = self.create_publisher(
            String,
            '/voice_commands',
            10
        )

        # Publisher for robot actions
        self.action_publisher = self.create_publisher(
            String,
            '/robot_actions',
            10
        )

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set energy threshold for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Start listening thread
        self.listening_thread = threading.Thread(target=self.listen_continuously)
        self.listening_thread.daemon = True
        self.listening_thread.start()

        self.get_logger().info('Voice-to-Action node initialized')

    def listen_continuously(self):
        """Continuously listen for voice commands"""
        with self.microphone as source:
            while rclpy.ok():
                try:
                    # Listen for audio
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5)

                    # Recognize speech
                    text = self.recognizer.recognize_google(audio)
                    self.get_logger().info(f'Recognized: {text}')

                    # Publish recognized text
                    text_msg = String()
                    text_msg.data = text
                    self.text_publisher.publish(text_msg)

                    # Process the command
                    self.process_command(text)

                except sr.WaitTimeoutError:
                    # No speech detected, continue listening
                    continue
                except sr.UnknownValueError:
                    self.get_logger().info('Could not understand audio')
                except sr.RequestError as e:
                    self.get_logger().error(f'Error with speech recognition service: {e}')
                except Exception as e:
                    self.get_logger().error(f'Unexpected error: {e}')

    def process_command(self, command_text):
        """Process the recognized command and map to robot action"""
        # Convert to lowercase for easier processing
        command = command_text.lower().strip()

        # Simple command mapping (in practice, use NLP techniques)
        if 'move forward' in command or 'go forward' in command:
            action = 'move_forward'
        elif 'move backward' in command or 'go back' in command:
            action = 'move_backward'
        elif 'turn left' in command:
            action = 'turn_left'
        elif 'turn right' in command:
            action = 'turn_right'
        elif 'stop' in command:
            action = 'stop'
        elif 'pick up' in command or 'grasp' in command:
            action = 'grasp_object'
        elif 'drop' in command or 'release' in command:
            action = 'release_object'
        else:
            self.get_logger().info(f'Unknown command: {command}')
            return

        # Publish the action
        action_msg = String()
        action_msg.data = action
        self.action_publisher.publish(action_msg)
        self.get_logger().info(f'Published action: {action}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceToActionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Natural Language Understanding

For more sophisticated voice command processing, we can use natural language processing techniques:

```python
import spacy
from dataclasses import dataclass
from typing import Optional
import re

@dataclass
class Command:
    action: str
    target: Optional[str] = None
    location: Optional[str] = None
    attributes: dict = None

class NaturalLanguageProcessor:
    def __init__(self):
        # Load spaCy model (install with: python -m spacy download en_core_web_sm)
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            self.nlp = None
            print("spaCy model not found. Install with: python -m spacy download en_core_web_sm")

    def parse_command(self, text: str) -> Optional[Command]:
        """Parse natural language command into structured format"""
        if not self.nlp:
            return self.fallback_parse(text)

        doc = self.nlp(text)

        # Extract action (verb)
        action = None
        for token in doc:
            if token.pos_ == "VERB":
                action = token.lemma_
                break

        # Extract target/object
        target = None
        for token in doc:
            if token.dep_ in ["dobj", "pobj"] and token.pos_ in ["NOUN", "PROPN"]:
                target = token.text
                break

        # Extract location
        location = None
        for token in doc:
            if token.dep_ == "pobj" and token.pos_ == "NOUN":
                # Check if this is a location preposition
                for child in token.head.children:
                    if child.text in ["to", "at", "in", "on"]:
                        location = token.text
                        break

        return Command(action=action, target=target, location=location)

    def fallback_parse(self, text: str) -> Command:
        """Simple fallback parsing without NLP"""
        # Extract action based on common verbs
        action = None
        text_lower = text.lower()

        if any(word in text_lower for word in ["move", "go", "drive", "navigate"]):
            action = "move"
        elif any(word in text_lower for word in ["pick", "grasp", "take", "grab"]):
            action = "grasp"
        elif any(word in text_lower for word in ["drop", "release", "put", "place"]):
            action = "release"
        elif any(word in text_lower for word in ["stop", "halt", "pause"]):
            action = "stop"
        elif any(word in text_lower for word in ["speak", "say", "tell"]):
            action = "speak"

        # Extract target using simple regex
        target_match = re.search(r'(?:to|the|a|an)\s+(\w+)', text_lower)
        target = target_match.group(1) if target_match else None

        return Command(action=action, target=target)

class AdvancedVoiceToActionNode(VoiceToActionNode):
    def __init__(self):
        super().__init__()
        self.nlp_processor = NaturalLanguageProcessor()

    def process_command(self, command_text):
        """Process command using NLP techniques"""
        command = self.nlp_processor.parse_command(command_text)

        if command.action:
            # Map NLP action to robot action
            robot_action = self.map_action(command.action, command.target, command.location)

            # Publish the action
            action_msg = String()
            action_msg.data = robot_action
            self.action_publisher.publish(action_msg)
            self.get_logger().info(f'Processed command: {command_text} -> {robot_action}')
        else:
            self.get_logger().info(f'Could not parse command: {command_text}')

    def map_action(self, action: str, target: str = None, location: str = None) -> str:
        """Map parsed command to robot action"""
        action_mapping = {
            'move': 'move_to_location',
            'go': 'move_to_location',
            'drive': 'move_to_location',
            'navigate': 'move_to_location',
            'pick': 'grasp_object',
            'grasp': 'grasp_object',
            'take': 'grasp_object',
            'grab': 'grasp_object',
            'drop': 'release_object',
            'release': 'release_object',
            'put': 'release_object',
            'place': 'release_object',
            'stop': 'stop_robot',
            'halt': 'stop_robot',
            'pause': 'stop_robot',
            'speak': 'speak_response',
            'say': 'speak_response',
            'tell': 'speak_response'
        }

        robot_action = action_mapping.get(action, f'unknown_{action}')

        # Add target and location information if available
        if target:
            robot_action += f'_{target}'
        if location:
            robot_action += f'_at_{location}'

        return robot_action
```

## Integration with Robot Systems

Voice commands can be integrated with various robot systems:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

class VoiceCommandExecutor(Node):
    def __init__(self):
        super().__init__('voice_command_executor')

        # Subscriber for voice commands
        self.command_subscriber = self.create_subscription(
            String,
            '/robot_actions',
            self.command_callback,
            10
        )

        # Action clients for different robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, ManipulationAction, 'manipulation_action')

        self.get_logger().info('Voice Command Executor initialized')

    def command_callback(self, msg):
        """Process incoming voice commands"""
        command = msg.data
        self.get_logger().info(f'Executing command: {command}')

        if command.startswith('move_to_location'):
            self.execute_navigation_command(command)
        elif command.startswith('grasp_object'):
            self.execute_manipulation_command(command)
        elif command.startswith('stop_robot'):
            self.execute_stop_command()
        elif command.startswith('speak_response'):
            self.execute_speak_command(command)
        else:
            self.get_logger().info(f'Unknown command: {command}')

    def execute_navigation_command(self, command):
        """Execute navigation-related voice commands"""
        # Parse location from command (simplified)
        location = self.parse_location_from_command(command)

        # In a real implementation, you would have a map of named locations
        # For now, we'll use a simple coordinate system
        if location == 'kitchen':
            target_pose = self.create_pose(2.0, 1.0, 0.0)  # x, y, theta
        elif location == 'living_room':
            target_pose = self.create_pose(-1.0, 2.0, 1.57)  # x, y, theta
        elif location == 'bedroom':
            target_pose = self.create_pose(0.0, -2.0, 0.0)  # x, y, theta
        else:
            self.get_logger().info(f'Unknown location: {location}')
            return

        # Send navigation goal
        self.send_navigation_goal(target_pose)

    def parse_location_from_command(self, command):
        """Extract location from command string"""
        # Simple keyword matching (in practice, use more sophisticated NLP)
        if 'kitchen' in command:
            return 'kitchen'
        elif 'living room' in command or 'living_room' in command:
            return 'living_room'
        elif 'bedroom' in command:
            return 'bedroom'
        elif 'office' in command:
            return 'office'
        else:
            return 'unknown'

    def create_pose(self, x, y, theta):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert theta to quaternion
        import math
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)

        return pose

    def send_navigation_goal(self, pose):
        """Send navigation goal to the robot"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        self.get_logger().info(f'Navigation feedback: {feedback_msg.feedback}')

    def execute_manipulation_command(self, command):
        """Execute manipulation-related voice commands"""
        # Parse object from command
        obj = self.parse_object_from_command(command)

        # In a real implementation, you would have object detection and manipulation planning
        # For now, we'll just log the command
        self.get_logger().info(f'Attempting to grasp object: {obj}')

    def parse_object_from_command(self, command):
        """Extract object from command string"""
        # Simple keyword matching
        known_objects = ['cup', 'book', 'ball', 'box', 'bottle']
        for obj in known_objects:
            if obj in command:
                return obj
        return 'unknown_object'

    def execute_stop_command(self):
        """Execute stop command"""
        self.get_logger().info('Stopping robot')
        # In a real implementation, send stop command to robot base

    def execute_speak_command(self, command):
        """Execute speech command"""
        # Extract text to speak from command
        # In a real implementation, use text-to-speech
        self.get_logger().info(f'Robot would speak: {command}')
```

## Hardware Integration Notes

**RTX Workstation**: Complex voice processing with large language models benefits from GPU acceleration.

**Jetson Orin Nano**: For edge deployment, consider using optimized speech recognition models and smaller language models.

**RealSense Integration**: Combine voice commands with visual feedback for enhanced interaction.

## Privacy and Security Considerations

Voice interfaces raise important privacy and security concerns:

- **Always-listening**: Implement privacy controls to disable voice processing when not needed
- **Data handling**: Ensure voice data is processed securely
- **Command authentication**: Verify that commands come from authorized users

## Summary

Voice-to-Action systems provide a natural interface for human-robot interaction. By combining speech recognition, natural language processing, and action mapping, robots can understand and execute commands expressed in natural language. Proper integration with robot systems enables flexible and intuitive control of robotic platforms.