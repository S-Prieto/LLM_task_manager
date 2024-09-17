from openai import OpenAI
from pydantic import BaseModel
from dotenv import load_dotenv
import os

# Load environment variables from the .env file
load_dotenv()

# Get the API key from environment variables
api_key = os.getenv("OPENAI_API_KEY")

# Initialize the OpenAI client with the API key
client = OpenAI(api_key=api_key)

# Define the function for step and action
step_action_function = {
    "type": "function",
    "function": {
        "name": "describe_process",
        "description": "Describes a process in terms of steps and actions",
        "parameters": {
            "type": "object",
            "properties": {
                "step": {
                    "type": "integer",
                    "description": "The step number in the process"
                },
                "action": {
                    "type": "string",
                    "enum": ["MOVE_S", "MOVE_B", "MOVE_C", "PICK", "BUILD", "CHARGE"],
                    "description": "The action to be performed at this step"
                }
            },
            "required": ["step", "action"],
            "additionalProperties": False
        },
        "strict": True
    }
}

# Create the assistant with structured output using the function
assistant = client.beta.assistants.create(
    name="LLM_task_manager_TEST",
    instructions="When describing a process, use the provided function to output each step as a numbered step with an associated action.",
    model="gpt-4o-mini",
    tools=[step_action_function]  # Add the function for steps and actions
)

# Store the assistant ID
assistant_id = assistant.id

# Create the thread
thread = client.beta.threads.create()

# Store the thread ID and assistant ID in a file
with open('thread_info.txt', 'w') as file:
    file.write(f"THREAD_ID={thread.id}\n")
    file.write(f"ASSISTANT_ID={assistant_id}\n")

print(f"Thread ID created: {thread.id}")
print(f"Assistant ID created: {assistant_id}")