from openai import OpenAI
from dotenv import load_dotenv
import os

# Load environment variables from the .env file
load_dotenv()

# Get the API key from environment variables
api_key = os.getenv("OPENAI_API_KEY")

# Load the thread ID from the file
with open('thread_info.txt', 'r') as file:
    lines = file.readlines()
    thread_id = lines[0].strip().split('=')[1]

# Initialize the OpenAI client with the API key
client = OpenAI(api_key=api_key)

# Retrieve the messages from the existing thread
messages = client.beta.threads.messages.list(thread_id=thread_id)

# Filter and display only the messages from the assistant role
for message in reversed(messages.data):
    if message.role == "assistant":
        print("Assistant: " + message.content[0].text.value)
