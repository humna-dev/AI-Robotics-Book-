---
sidebar_position: 12
---

# Cognitive Planning with LLMs

This topic covers the integration of Large Language Models (LLMs) with robotic cognitive planning systems, enabling robots to perform complex reasoning and planning tasks using natural language.

## Cognitive Planning Overview

Cognitive planning in robotics involves high-level reasoning about tasks, goals, and strategies. By integrating Large Language Models (LLMs), robots can understand natural language instructions, reason about complex tasks, and generate detailed action plans.

### Components of LLM-Based Cognitive Planning

1. **Task Understanding**: Interpreting natural language instructions
2. **World Modeling**: Representing the environment and robot capabilities
3. **Plan Generation**: Creating sequences of actions to achieve goals
4. **Plan Execution**: Monitoring and adapting plans during execution
5. **Learning**: Updating models based on experience

## LLM Integration Architecture

### Basic LLM Interface

```python
import openai
import json
from dataclasses import dataclass
from typing import List, Dict, Any
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

@dataclass
class ActionStep:
    action: str
    parameters: Dict[str, Any]
    description: str

@dataclass
class TaskPlan:
    steps: List[ActionStep]
    context: str
    success_criteria: List[str]

class LLMCognitivePlanner(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # Publisher for generated plans
        self.plan_publisher = self.create_publisher(
            String,
            '/cognitive_plans',
            10
        )

        # Subscriber for natural language commands
        self.command_subscriber = self.create_subscription(
            String,
            '/natural_language_commands',
            self.command_callback,
            10
        )

        # Set OpenAI API key (in practice, use secure configuration)
        openai.api_key = "your-openai-api-key-here"

        self.get_logger().info('LLM Cognitive Planner initialized')

    def command_callback(self, msg):
        """Process natural language command and generate plan"""
        command = msg.data
        self.get_logger().info(f'Processing command: {command}')

        try:
            # Generate plan using LLM
            plan = self.generate_plan(command)

            # Publish the plan
            plan_json = json.dumps({
                'command': command,
                'plan': [step.__dict__ for step in plan.steps],
                'context': plan.context,
                'success_criteria': plan.success_criteria
            })

            plan_msg = String()
            plan_msg.data = plan_json
            self.plan_publisher.publish(plan_msg)

            self.get_logger().info(f'Generated plan with {len(plan.steps)} steps')

        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')

    def generate_plan(self, command: str) -> TaskPlan:
        """Generate a task plan using LLM"""
        # Define the robot's capabilities
        capabilities = [
            "move_to(location)",
            "grasp_object(object)",
            "release_object()",
            "detect_object(object_type)",
            "navigate_to_object(object_type)",
            "ask_for_help()",
            "wait_for(duration)"
        ]

        # Create prompt for LLM
        prompt = f"""
        You are a cognitive planning system for a robot. Generate a detailed plan to execute the following command: "{command}"

        Robot capabilities:
        {", ".join(capabilities)}

        Available locations: kitchen, living_room, bedroom, office, hallway

        Please return the plan as a JSON object with the following structure:
        {{
            "steps": [
                {{
                    "action": "action_name",
                    "parameters": {{"param_name": "param_value"}},
                    "description": "Brief description of the step"
                }}
            ],
            "context": "Relevant context for the task",
            "success_criteria": ["list", "of", "success", "criteria"]
        }}

        Make sure the plan is executable with the given capabilities and handles potential failures gracefully.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=1000
            )

            # Parse the response
            response_text = response.choices[0].message['content'].strip()

            # Extract JSON from response (in case the LLM includes extra text)
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                plan_data = json.loads(json_str)

                # Convert to TaskPlan object
                steps = [ActionStep(**step) for step in plan_data['steps']]
                return TaskPlan(
                    steps=steps,
                    context=plan_data['context'],
                    success_criteria=plan_data['success_criteria']
                )

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing LLM response: {e}')
        except Exception as e:
            self.get_logger().error(f'Error calling LLM: {e}')

        # Return a default plan if LLM fails
        return TaskPlan(
            steps=[ActionStep(
                action="ask_for_help",
                parameters={},
                description="Unable to generate plan, requesting human assistance"
            )],
            context="Plan generation failed",
            success_criteria=["human_intervention"]
        )
```

## Context-Aware Planning

LLMs can incorporate contextual information to make better planning decisions:

```python
class ContextAwarePlanner(LLMCognitivePlanner):
    def __init__(self):
        super().__init__()
        self.world_state = {
            'robot_position': 'unknown',
            'available_objects': [],
            'obstacles': [],
            'time_of_day': 'unknown',
            'human_presence': False
        }

    def update_world_state(self, state_update: Dict[str, Any]):
        """Update the world state with new information"""
        self.world_state.update(state_update)

    def generate_plan_with_context(self, command: str) -> TaskPlan:
        """Generate a plan considering current world state"""
        # Get current context
        context = self.get_current_context()

        # Create enhanced prompt with context
        prompt = f"""
        You are a cognitive planning system for a robot. Generate a detailed plan to execute the following command: "{command}"

        Current context:
        - Robot position: {context['robot_position']}
        - Available objects: {context['available_objects']}
        - Obstacles: {context['obstacles']}
        - Time of day: {context['time_of_day']}
        - Human presence: {context['human_presence']}

        Robot capabilities:
        - move_to(location)
        - grasp_object(object)
        - release_object()
        - detect_object(object_type)
        - navigate_to_object(object_type)
        - ask_for_help()
        - wait_for(duration)
        - speak(text)

        Available locations: kitchen, living_room, bedroom, office, hallway

        Please return the plan as a JSON object with the following structure:
        {{
            "steps": [
                {{
                    "action": "action_name",
                    "parameters": {{"param_name": "param_value"}},
                    "description": "Brief description of the step"
                }}
            ],
            "context": "Relevant context for the task",
            "success_criteria": ["list", "of", "success", "criteria"]
        }}

        Consider the current context when generating the plan. For example:
        - If it's nighttime, consider lighting needs
        - If humans are present, consider safety and politeness
        - If the robot is in a different location than needed, plan navigation first
        - If required objects are not available, consider alternatives
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=1200
            )

            response_text = response.choices[0].message['content'].strip()
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                plan_data = json.loads(json_str)

                steps = [ActionStep(**step) for step in plan_data['steps']]
                return TaskPlan(
                    steps=steps,
                    context=plan_data['context'],
                    success_criteria=plan_data['success_criteria']
                )

        except Exception as e:
            self.get_logger().error(f'Error generating contextual plan: {e}')

        # Fallback plan
        return TaskPlan(
            steps=[ActionStep(
                action="ask_for_help",
                parameters={},
                description="Unable to generate contextual plan, requesting human assistance"
            )],
            context="Contextual plan generation failed",
            success_criteria=["human_intervention"]
        )

    def get_current_context(self) -> Dict[str, Any]:
        """Get the current world context"""
        return self.world_state.copy()
```

## Plan Execution and Monitoring

LLMs can also help monitor plan execution and adapt when needed:

```python
class PlanExecutionMonitor(Node):
    def __init__(self):
        super().__init__('plan_execution_monitor')

        # Subscriber for plan execution status
        self.status_subscriber = self.create_subscription(
            String,
            '/plan_execution_status',
            self.status_callback,
            10
        )

        # Publisher for plan adaptations
        self.adaptation_publisher = self.create_publisher(
            String,
            '/plan_adaptations',
            10
        )

        # Track current plan
        self.current_plan = None
        self.current_step_index = 0

    def status_callback(self, msg):
        """Handle plan execution status updates"""
        try:
            status_data = json.loads(msg.data)
            status = status_data['status']  # 'success', 'failure', 'progress'
            step_index = status_data['step_index']
            details = status_data.get('details', '')

            if status == 'failure':
                self.handle_plan_failure(step_index, details)
            elif status == 'progress':
                self.handle_plan_progress(step_index, details)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid status message format')

    def handle_plan_failure(self, step_index: int, details: str):
        """Handle plan execution failure using LLM reasoning"""
        if not self.current_plan:
            return

        # Get the failed step
        failed_step = self.current_plan.steps[step_index] if step_index < len(self.current_plan.steps) else None

        # Create prompt for LLM to suggest adaptation
        prompt = f"""
        A robot's plan execution has failed. Here are the details:

        Failed step: {failed_step.description if failed_step else 'Unknown'}
        Failure details: {details}
        Current plan: {[step.description for step in self.current_plan.steps]}
        Current step index: {step_index}

        Available robot capabilities:
        - move_to(location)
        - grasp_object(object)
        - release_object()
        - detect_object(object_type)
        - navigate_to_object(object_type)
        - ask_for_help()
        - wait_for(duration)
        - speak(text)
        - retry_step(step_index)

        Suggest an appropriate adaptation to the plan. Return the response as JSON with the following structure:
        {{
            "action": "retry|skip|modify|abort|ask_help",
            "new_plan": [list of new steps if modifying],
            "reason": "Explanation for the adaptation"
        }}
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.5,
                max_tokens=500
            )

            response_text = response.choices[0].message['content'].strip()
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                adaptation = json.loads(json_str)

                # Publish adaptation
                adaptation_msg = String()
                adaptation_msg.data = json.dumps(adaptation)
                self.adaptation_publisher.publish(adaptation_msg)

                self.get_logger().info(f'Plan adaptation suggested: {adaptation["action"]}')

        except Exception as e:
            self.get_logger().error(f'Error generating plan adaptation: {e}')

    def handle_plan_progress(self, step_index: int, details: str):
        """Handle plan execution progress"""
        self.get_logger().info(f'Plan progress: step {step_index}, details: {details}')
```

## Multi-Modal Integration

LLMs can be combined with vision systems for more sophisticated cognitive planning:

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MultiModalCognitivePlanner(ContextAwarePlanner):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()

        # Subscriber for camera images
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Store recent images for context
        self.recent_image = None

    def image_callback(self, msg):
        """Process camera images for multi-modal planning"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.recent_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def generate_plan_with_vision(self, command: str) -> TaskPlan:
        """Generate plan incorporating visual information"""
        # In a real implementation, you would use vision-language models
        # like CLIP or specialized robotic vision systems
        # For this example, we'll simulate by describing the image content

        image_description = self.describe_image_content()

        prompt = f"""
        You are a cognitive planning system for a robot. Generate a detailed plan to execute the following command: "{command}"

        Current context:
        - Robot position: {self.world_state['robot_position']}
        - Available objects: {self.world_state['available_objects']}
        - Time of day: {self.world_state['time_of_day']}
        - Human presence: {self.world_state['human_presence']}

        Current visual scene: {image_description}

        Robot capabilities:
        - move_to(location)
        - grasp_object(object)
        - release_object()
        - detect_object(object_type)
        - navigate_to_object(object_type)
        - ask_for_help()
        - wait_for(duration)
        - speak(text)

        Available locations: kitchen, living_room, bedroom, office, hallway

        Please return the plan as a JSON object with the following structure:
        {{
            "steps": [
                {{
                    "action": "action_name",
                    "parameters": {{"param_name": "param_value"}},
                    "description": "Brief description of the step"
                }}
            ],
            "context": "Relevant context for the task",
            "success_criteria": ["list", "of", "success", "criteria"]
        }}

        Consider both the visual scene and other context when generating the plan.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4-vision-preview",  # Use vision-capable model
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=1200
            )

            response_text = response.choices[0].message['content'].strip()
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                plan_data = json.loads(json_str)

                steps = [ActionStep(**step) for step in plan_data['steps']]
                return TaskPlan(
                    steps=steps,
                    context=plan_data['context'],
                    success_criteria=plan_data['success_criteria']
                )

        except Exception as e:
            self.get_logger().error(f'Error generating vision-aware plan: {e}')

        return self.generate_plan_with_context(command)

    def describe_image_content(self) -> str:
        """Describe the content of the recent image (simulated)"""
        # In a real implementation, this would use computer vision
        # to identify objects, people, and scene elements
        return "The robot sees a kitchen with a table, chairs, and a counter. There is a red cup on the table."
```

## Safety and Ethical Considerations

LLM-based cognitive planning raises important safety and ethical questions:

### Safety Measures

1. **Action Validation**: Verify that LLM-generated actions are safe before execution
2. **Constraint Checking**: Ensure plans respect safety constraints
3. **Human Oversight**: Maintain human-in-the-loop for critical decisions
4. **Fail-Safe Mechanisms**: Implement robust fallback procedures

```python
class SafeCognitivePlanner(MultiModalCognitivePlanner):
    def __init__(self):
        super().__init__()
        self.safety_constraints = [
            "don't enter restricted areas",
            "don't harm humans or animals",
            "don't damage property",
            "follow traffic rules",
            "respect privacy"
        ]

    def validate_plan(self, plan: TaskPlan, command: str) -> bool:
        """Validate that the plan is safe and appropriate"""
        # Create validation prompt
        prompt = f"""
        Validate the following robot plan for safety and appropriateness:

        Original command: {command}
        Generated plan: {[step.description for step in plan.steps]}
        Safety constraints: {self.safety_constraints}

        Return a JSON object with the following structure:
        {{
            "is_safe": true/false,
            "issues": ["list", "of", "safety", "issues"],
            "suggestions": ["list", "of", "improvements"]
        }}
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,  # Low temperature for more consistent validation
                max_tokens=500
            )

            response_text = response.choices[0].message['content'].strip()
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                validation = json.loads(json_str)

                if not validation.get('is_safe', True):
                    self.get_logger().warn(f'Safety issues found: {validation.get("issues", [])}')
                    return False

                return True

        except Exception as e:
            self.get_logger().error(f'Error validating plan: {e}')

        # If validation fails, default to safe behavior
        return False
```

## Hardware Integration Notes

**RTX Workstation**: LLM-based cognitive planning can benefit from GPU acceleration, especially for real-time processing and vision-language models.

**Jetson Orin Nano**: For edge deployment, consider using smaller, optimized language models or hybrid approaches that combine rule-based systems with lightweight LLMs.

**RealSense Integration**: Combine LLM reasoning with RealSense depth and RGB data for enhanced scene understanding.

## Summary

Cognitive planning with LLMs enables robots to perform complex reasoning tasks using natural language. By integrating LLMs with robotic systems, robots can understand high-level commands, generate detailed action plans, adapt to changing conditions, and incorporate multi-modal information. However, safety and ethical considerations are paramount when deploying such systems in real-world environments.