import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyrobosim_msgs.msg import RobotState
from openai import OpenAI
from dotenv import load_dotenv
import os
import time

class OpenAIBridge(Node):
    def __init__(self):
        super().__init__('openai_bridge')
        self.robot_states = {}
        
        # Create subscribers for each robot's state
        self.create_subscription(RobotState, 'robot_1/robot_state', self.robot_state_callback, 10)
        self.create_subscription(RobotState, 'robot_2/robot_state', self.robot_state_callback, 10)
        
        # Create publisher for schedule commands
        self.schedule_publisher = self.create_publisher(String, 'schedule_commands', 10)
        
        # Timer to periodically send state to OpenAI
        self.create_timer(10.0, self.send_state_to_openai)  # every 10 seconds
        
        # OpenAI setup
        self.setup_openai()

    def setup_openai(self):
        # First, check if the environment variable ROS2GPT_ENV_PATH is set for development mode
        env_path = os.getenv('ROS2GPT_ENV_PATH')
        
        if not env_path:
            # If not in development mode, use the installed package directory
            package_dir = os.path.dirname(os.path.realpath(__file__))  # Adjust to current script location
            self.get_logger().info(f"Package directory: {package_dir}")
            env_path = os.path.join(package_dir, '.env')  # Default to the installed path
        
        self.get_logger().info(f"Looking for .env file at: {env_path}")
        
        if os.path.exists(env_path):
            self.get_logger().info(f".env file found at {env_path}")
            load_dotenv(env_path)
            with open(env_path, 'r') as env_file:
                env_contents = env_file.read()
                self.get_logger().info(f".env file contents (first 10 characters): {env_contents[:10]}...")
        else:
            self.get_logger().error(f".env file not found at {env_path}")
            raise FileNotFoundError(f".env file not found at {env_path}")

        # Get the API key from environment variables
        self.api_key = os.getenv("OPENAI_API_KEY")
        
        if self.api_key:
            self.get_logger().info(f"API key loaded successfully: {self.api_key[:5]}...")
        else:
            self.get_logger().error("OpenAI API key not found in .env file.")
            raise ValueError("OpenAI API key not found")
        
        self.client = OpenAI(api_key=self.api_key)
        self.get_logger().info("OpenAI client initialized successfully")

        # Load thread info
        thread_info_path = os.path.join(os.path.dirname(env_path), 'thread_info.txt')  # Adjusted path for thread_info.txt
        self.get_logger().info(f"Looking for thread_info.txt at: {thread_info_path}")
        if os.path.exists(thread_info_path):
            with open(thread_info_path, 'r') as file:
                content = file.read()
                self.get_logger().info(f"thread_info.txt contents: {content}")
                lines = content.splitlines()
                self.thread_id = lines[0].strip().split('=')[1]
                self.assistant_id = lines[1].strip().split('=')[1]
            self.get_logger().info(f"Loaded thread_info.txt: Thread ID={self.thread_id}, Assistant ID={self.assistant_id}")
        else:
            self.get_logger().error(f"thread_info.txt not found at {thread_info_path}")
            raise FileNotFoundError(f"thread_info.txt not found at {thread_info_path}")

    def robot_state_callback(self, msg):
        self.robot_states[msg.name] = msg
        self.get_logger().debug(f'Received state for {msg.name}:')
        self.get_logger().debug(f'  Position: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')
        self.get_logger().debug(f'  Current Location: {msg.current_location}')
        self.get_logger().debug(f'  Battery Level: {msg.battery_level}')
        self.get_logger().debug(f'  Holding Object: {msg.holding_object}')
        self.get_logger().debug(f'  Manipulated Object: {msg.manipulated_object}')
        
    def send_state_to_openai(self):
        if len(self.robot_states) < 2:
            self.get_logger().info('Waiting for all robot states...')
            return

        # Format the state data
        formatted_state = (
            f"{{[{self.get_location(self.robot_states['robot_1'])}, "
            f"{self.get_location(self.robot_states['robot_2'])}], "
            f"[{self.get_cargo(self.robot_states['robot_1'])}, "
            f"{self.get_cargo(self.robot_states['robot_2'])}], "
            f"[{self.get_battery(self.robot_states['robot_1'])}, "
            f"{self.get_battery(self.robot_states['robot_2'])}]}}"
        )

        self.get_logger().info(f'Sending state to OpenAI: {formatted_state}')

        try:
            # Send to OpenAI
            message = self.client.beta.threads.messages.create(
                thread_id=self.thread_id,
                role="user",
                content=formatted_state
            )

            # Run assistant
            run = self.client.beta.threads.runs.create(
                thread_id=self.thread_id,
                assistant_id=self.assistant_id
            )

            # Wait for and retrieve the assistant's response
            while True:
                run_status = self.client.beta.threads.runs.retrieve(
                    thread_id=self.thread_id,
                    run_id=run.id
                )
                if run_status.status == 'completed':
                    break
                time.sleep(1)

            messages = self.client.beta.threads.messages.list(thread_id=self.thread_id)
            assistant_message = next((msg for msg in messages.data if msg.role == "assistant"), None)

            if assistant_message:
                # Publish the assistant's response to the schedule_commands topic
                msg = String()
                msg.data = assistant_message.content[0].text.value
                self.schedule_publisher.publish(msg)
                self.get_logger().info(f'Published schedule: {msg.data}')
            else:
                self.get_logger().warn('No response received from assistant')
        except Exception as e:
            self.get_logger().error(f'Error in OpenAI communication: {str(e)}')

    def get_location(self, robot_state):
        location = robot_state.current_location
        if location == 'charging':
            return 'C'
        elif location == 'pallet0_pallet':
            return 'S'
        elif location == 'pallet1_pallet':
            return 'B'
        else:
            return 'T'  # 'T' for traveling if it's in none of the above

    def get_cargo(self, robot_state):
        return 1 if robot_state.holding_object else 0

    def get_battery(self, robot_state):
        return robot_state.battery_level

def main(args=None):
    rclpy.init(args=args)
    openai_bridge = OpenAIBridge()
    rclpy.spin(openai_bridge)
    openai_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

