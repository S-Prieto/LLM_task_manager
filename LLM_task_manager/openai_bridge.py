import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyrobosim_msgs.msg import RobotState
from openai import OpenAI
from dotenv import load_dotenv
import os
import json
import time

class OpenAIBridge(Node):
    def __init__(self):
        super().__init__('openai_bridge')
        self.robot_states = {}
        self.max_retries = 5  # Max number of retries to handle suggestions

        # Create subscribers for each robot's state
        self.create_subscription(RobotState, 'robot_1/robot_state', self.robot_state_callback, 10)
        self.create_subscription(RobotState, 'robot_2/robot_state', self.robot_state_callback, 10)

        # Create publisher for schedule commands
        self.schedule_publisher = self.create_publisher(String, 'schedule_commands', 10)

        # Timer to periodically send state to OpenAI
        self.create_timer(10.0, self.send_state_to_openai)  # every 10 seconds

        # OpenAI setup
        self.setup_openai()

        # Initialize placed_beams and step number
        self.placed_beams = 0
        self.step_number = 1

    def setup_openai(self):
        """Set up OpenAI client, assistants, and thread."""
        load_dotenv()

        # Get the API key from environment variables
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            self.get_logger().error("OpenAI API key not found in environment variables.")
            raise ValueError("OpenAI API key not found")
        # Initialize OpenAI client
        self.client = OpenAI(api_key=self.api_key)
        self.get_logger().info("OpenAI client initialized successfully")

        # Load thread info
        thread_info_path = '/home/sesath/ros2gpt/src/llm_task_manager/llm_task_manager/thread_info.txt'
        if os.path.exists(thread_info_path):
            with open(thread_info_path, 'r') as file:
                content = file.read()
                self.get_logger().info(f"thread_info.txt contents: {content}")
                lines = content.strip().split('\n')
                self.thread_id = lines[0].split('=')[1].strip()
                self.instruction_assistant_id = lines[1].split('=')[1].strip()
                self.supervisor_assistant_id = lines[2].split('=')[1].strip()
            self.get_logger().info(f"Loaded thread info: Thread ID={self.thread_id}, Instruction Assistant ID={self.instruction_assistant_id}, Supervisor Assistant ID={self.supervisor_assistant_id}")

            # Store assistant names for logging
            self.assistant_names = {
                self.instruction_assistant_id: "Instruction Generator",
                self.supervisor_assistant_id: "Supervisor"
            }
        else:
            self.get_logger().error(f"thread_info.txt not found at {thread_info_path}")
            raise FileNotFoundError(f"thread_info.txt not found at {thread_info_path}")

    def robot_state_callback(self, msg):
        """Callback for receiving robot state messages."""
        self.robot_states[msg.name] = msg
        self.get_logger().debug(f'Received state for {msg.name}: '
                                f'Position: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}, '
                                f'Current Location: {msg.current_location}, '
                                f'Battery Level: {msg.battery_level}, '
                                f'Holding Object: {msg.holding_object}, '
                                f'Manipulated Object: {msg.manipulated_object}')

    def send_state_to_openai(self):
        """Send the current state to OpenAI using the assistant runs."""
        if len(self.robot_states) < 2:
            self.get_logger().info('Waiting for all robot states...')
            return

        # Format the state data
        formatted_state = self.format_state_for_openai()
        self.get_logger().info(f'Sending state to OpenAI: {formatted_state}')

        retry_count = 0
        supervisor_feedback = None

        while retry_count < self.max_retries:
            # Step 1: Send to Instruction Assistant
            instruction_response = self.get_instructions_from_openai(formatted_state, supervisor_feedback)

            if not instruction_response:
                self.get_logger().error('No response from instruction assistant.')
                return

            instruction = instruction_response  # Instruction is a string

            # Step 2: Send to Supervisor Assistant
            supervisor_response = self.evaluate_instructions_with_supervisor(formatted_state, instruction)

            if not supervisor_response:
                self.get_logger().error('No response from supervisor assistant.')
                return

            if supervisor_response.strip().upper() == 'ACCEPTED':
                # Instructions are acceptable, publish them
                self.publish_instructions(instruction)
                # Update internal state
                self.update_state_from_instruction(instruction)
                return
            else:
                # Supervisor provided feedback, inform the instruction assistant in the next iteration
                supervisor_feedback = supervisor_response
                self.get_logger().info(f'Supervisor feedback: {supervisor_feedback}')
                retry_count += 1

        self.get_logger().error('Max retries reached. Could not generate valid instructions.')

    def run_assistant(self, assistant_id, assistant_input):
        """Run the assistant and retrieve the response."""
        try:
            assistant_name = self.assistant_names.get(assistant_id, "Unknown Assistant")

            # Send message to assistant
            message = self.client.beta.threads.messages.create(
                thread_id=self.thread_id,
                role="user",
                content=assistant_input
            )

            # Run assistant
            run = self.client.beta.threads.runs.create(
                thread_id=self.thread_id,
                assistant_id=assistant_id
            )

            # Wait for run to complete
            while True:
                run_status = self.client.beta.threads.runs.retrieve(
                    thread_id=self.thread_id,
                    run_id=run.id
                )
                if run_status.status in ['succeeded', 'completed']:
                    break
                elif run_status.status == 'failed':
                    self.get_logger().error(f"Run failed for assistant '{assistant_name}': {run_status.error}")
                    return None
                time.sleep(1)

            # Retrieve messages specific to the run
            run_messages = self.client.beta.threads.messages.list(
                thread_id=self.thread_id,
                run_id=run.id
            )

            # Find the assistant's message
            assistant_message = next((msg for msg in run_messages.data if msg.role == "assistant"), None)

            if assistant_message:
                assistant_response_content = assistant_message.content
                # Extract text value from content
                if isinstance(assistant_response_content, list):
                    # Assuming the content is a list of TextContentBlock objects
                    text_values = []
                    for content_block in assistant_response_content:
                        if hasattr(content_block, 'text') and hasattr(content_block.text, 'value'):
                            text_values.append(content_block.text.value)
                    assistant_response = '\n'.join(text_values)
                else:
                    assistant_response = str(assistant_response_content)
                return assistant_response
            else:
                self.get_logger().warn(f"No assistant message found for assistant '{assistant_name}'")
                return None
        except Exception as e:
            assistant_name = self.assistant_names.get(assistant_id, "Unknown Assistant")
            self.get_logger().error(f'Error in OpenAI communication with assistant "{assistant_name}": {str(e)}')
            return None

    def prepare_instruction_input(self, formatted_state, supervisor_feedback):
        """Prepare input for the instruction assistant."""
        assistant_input = f"""
Feedback:
{formatted_state}
Placed_beams: {self.placed_beams}
Step: {self.step_number}
"""
        if supervisor_feedback:
            assistant_input += f"\nSupervisor feedback: {supervisor_feedback}\n"
        return assistant_input

    def prepare_supervisor_input(self, formatted_state, instruction):
        """Prepare input for the supervisor assistant."""
        supervisor_input = f"""
Robot State:
{formatted_state}
Placed_beams: {self.placed_beams}
Step: {self.step_number}

Instructions:
{instruction}
"""
        return supervisor_input

    def get_instructions_from_openai(self, formatted_state, supervisor_feedback=None):
        """Send the state (and optional feedback) to the instruction assistant and return instructions."""
        assistant_input = self.prepare_instruction_input(formatted_state, supervisor_feedback)

        assistant_response = self.run_assistant(
            assistant_id=self.instruction_assistant_id,
            assistant_input=assistant_input
        )

        if not assistant_response:
            self.get_logger().error('No response from instruction assistant.')
            return None

        self.get_logger().info(f'Received instructions from instruction assistant: {assistant_response}')
        return assistant_response  # Return as string

    def evaluate_instructions_with_supervisor(self, formatted_state, instruction):
        """Send the generated instructions to the supervisor for validation."""
        self.get_logger().info('Sending instructions to supervisor for evaluation...')

        supervisor_input = self.prepare_supervisor_input(formatted_state, instruction)

        supervisor_response = self.run_assistant(
            assistant_id=self.supervisor_assistant_id,
            assistant_input=supervisor_input
        )

        if supervisor_response:
            self.get_logger().info(f'Supervisor evaluation: {supervisor_response}')
            return supervisor_response
        else:
            self.get_logger().error('No response from supervisor assistant.')
            return None

    def publish_instructions(self, instruction):
        """Publish the final validated instructions to the schedule_commands topic."""
        msg = String()
        msg.data = instruction  # Assuming instruction is a string
        self.schedule_publisher.publish(msg)
        self.get_logger().info(f'Published validated instructions: {msg.data}')

    def update_state_from_instruction(self, instruction):
        """Update internal state based on the instruction."""
        try:
            instruction_data = json.loads(instruction)
            self.step_number = instruction_data.get('step', self.step_number + 1)
            self.placed_beams = instruction_data.get('placed_beams', self.placed_beams)
            # Update other state variables as needed
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse instruction JSON: {e}')

    def format_state_for_openai(self):
        """Helper function to format the robot state for OpenAI assistant."""
        locations = [self.get_location(self.robot_states['robot_1']),
                     self.get_location(self.robot_states['robot_2'])]
        internal_cargo = [self.get_cargo(self.robot_states['robot_1']),
                          self.get_cargo(self.robot_states['robot_2'])]
        remaining_battery = [self.get_battery(self.robot_states['robot_1']),
                             self.get_battery(self.robot_states['robot_2'])]

        formatted_state = {
            "locations": locations,
            "internal_cargo": internal_cargo,
            "remaining_battery": remaining_battery
        }

        formatted_state_json = json.dumps(formatted_state, indent=4)
        return formatted_state_json

    def get_location(self, robot_state):
        """Helper function to convert location to C, S, B, T."""
        location = robot_state.current_location.lower()
        if location in ['charging_area', 'charging']:
            return 'C'
        elif location in ['storage_area', 'storage', 'pallet0_pallet']:
            return 'S'
        elif location in ['building_area', 'building', 'pallet1_pallet']:
            return 'B'
        else:
            return 'T'  # 'T' for traveling if it's none of the above

    def get_cargo(self, robot_state):
        """Helper function to determine cargo status (1 if holding, 0 if not)."""
        return 1 if robot_state.holding_object else 0

    def get_battery(self, robot_state):
        """Helper function to get the robot's battery level."""
        return robot_state.battery_level

def main(args=None):
    rclpy.init(args=args)
    openai_bridge = OpenAIBridge()
    rclpy.spin(openai_bridge)
    openai_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
