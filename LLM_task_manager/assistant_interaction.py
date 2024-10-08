from openai import OpenAI
from dotenv import load_dotenv
import os
import time

# Load environment variables from the .env file
load_dotenv()

# Get the API key from environment variables
api_key = os.getenv("OPENAI_API_KEY")

# Initialize the OpenAI client with the API key
client = OpenAI(api_key=api_key)

# Create the assistant
assistant = client.beta.assistants.create(
    name="LLM_task_manager_TEST",
    instructions="Whatever number you're given, multiply it by 10. Do not output any message, just the number",
    tools=[{"type": "code_interpreter"}],
    model="gpt-4o-mini"
)

# Create the thread
thread = client.beta.threads.create()

# Add message to thread
message = client.beta.threads.messages.create(
    thread_id=thread.id,
    role="user",
    content="2.35"
)

# Run assistant
run = client.beta.threads.runs.create(
    thread_id=thread.id,
    assistant_id=assistant.id
)

# Poll for the assistant's response with a timeout
timeout = 10  # Maximum time to wait in seconds
poll_interval = 1  # Time between checks in seconds
elapsed_time = 0

while elapsed_time < timeout:
    messages = client.beta.threads.messages.list(thread_id=thread.id)
    
    # Check if there is a message from the assistant
    assistant_messages = [msg for msg in messages.data if msg.role == "assistant"]
    
    if assistant_messages:
        # If there's at least one assistant message, display all messages
        for message in reversed(messages.data):
            print(message.role + ": " + message.content[0].text.value)
        break
    
    # Wait before the next check
    time.sleep(poll_interval)
    elapsed_time += poll_interval

# If the loop exits without finding an assistant message, handle it
if not assistant_messages:
    print("Error: Assistant did not respond within the timeout period.")
