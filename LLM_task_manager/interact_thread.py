from openai import OpenAI
from dotenv import load_dotenv
import os
import time

# Load environment variables from the .env file
load_dotenv()

# Get the API key from environment variables
api_key = os.getenv("OPENAI_API_KEY")

# Load the thread ID and assistant ID from the file
with open('thread_info.txt', 'r') as file:
    lines = file.readlines()
    thread_id = lines[0].strip().split('=')[1]
    assistant_id = lines[1].strip().split('=')[1]

# Initialize the OpenAI client with the API key
client = OpenAI(api_key=api_key)

# Add message to the existing thread
message = client.beta.threads.messages.create(
    thread_id=thread_id,
    role="user",
    content="How do you draw a circle"
)

# Run assistant
run = client.beta.threads.runs.create(
    thread_id=thread_id,
    assistant_id=assistant_id  # Use the stored assistant_id
)

# Poll for the assistant's response with a timeout
timeout = 20  # Maximum time to wait in seconds
poll_interval = 1  # Time between checks in seconds
elapsed_time = 0

while elapsed_time < timeout:
    messages = client.beta.threads.messages.list(thread_id=thread_id)
    
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
