from openai import OpenAI
from dotenv import load_dotenv
import os

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
    response_format="text"
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
