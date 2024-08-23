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
    name = "LLM_task_manager_TEST",
    instructions="Whatever number you're given, multiply it by 10. Do not output any message, just the number",
    tools = [{"type": "code_interpreter"}],
    model = "gpt-4o-mini"
)

# Create the thread
thread = client.beta.threads.create()
# print(thread)

# Add message to thread
message = client.beta.threads.messages.create(
    thread_id = thread.id,
    role = "user", 
    content = "4"
)

# print(message)

# Run assistant

run = client.beta.threads.runs.create(
    thread_id = thread.id,
    assistant_id = assistant.id
)

# Display response

run = client.beta.threads.runs.retrieve(
    thread_id = thread.id,
    run_id = run.id
)

messages = client.beta.threads.messages.list(
    thread_id = thread.id
)

for message in reversed(messages.data):
    print(message.role + ": " + message.content[0].text.value)