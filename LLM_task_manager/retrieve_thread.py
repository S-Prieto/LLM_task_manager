from openai import OpenAI
from dotenv import load_dotenv
import os

# Load environment variables from the .env file
load_dotenv()

# Get the API key from environment variables
api_key = os.getenv("OPENAI_API_KEY")

# Load the thread ID and assistant ID from the file
with open('thread_info.txt', 'r') as file:
    lines = file.readlines()
    thread_id = 'thread_ByTosT0G5PqlPWF2DoxAOqJn'
    assistant_id = 'asst_p3KXhIBPAThM0ixa6VmjXqiR'
# Initialize the OpenAI client with the API key
client = OpenAI(api_key=api_key)

# Create and poll the run for completion
run = client.beta.threads.runs.create_and_poll(
    thread_id=thread_id,
    assistant_id=assistant_id,
)

# Check the run status and fetch messages if completed
if run.status == 'completed':
    messages = client.beta.threads.messages.list(thread_id=thread_id)
    print(messages)
else:
    print(f"Run status: {run.status}")

# Define the list to store tool outputs
tool_outputs = []

# # Check if there is a required action before accessing tool_calls
# if hasattr(run.required_action, 'submit_tool_outputs'):
#     # Loop through each tool call in the required action section of the run
#     for tool in run.required_action.submit_tool_outputs.tool_calls:
#         if tool.function.name == "describe_process":
#             # Assuming the output you want to simulate for testing
#             tool_outputs.append({
#                 "tool_call_id": tool.id,
#                 "output": "Step 1: Start action X"
#             })
# else:
#     print("No tool calls to handle.")

# # Submit the tool outputs if any are collected
# if tool_outputs:
#     try:
#         run = client.beta.threads.runs.submit_tool_outputs_and_poll(
#             thread_id=thread_id,
#             run_id=run.id,
#             tool_outputs=tool_outputs
#         )
#         print("Tool outputs submitted successfully.")
#     except Exception as e:
#         print(f"Failed to submit tool outputs: {e}")
# else:
#     print("No tool outputs to submit.")

# # Check the status after tool outputs are submitted
# if run.status == 'completed':
#     messages = client.beta.threads.messages.list(thread_id=thread_id)
#     print(messages)
# else:
#     print(f"Run status: {run.status}")