from openai import OpenAI
from dotenv import load_dotenv
import os

# Load environment variables from the .env file
load_dotenv()

# Get the API key from environment variables
api_key = os.getenv("OPENAI_API_KEY")

# Initialize the OpenAI client with the API key
client = OpenAI(api_key=api_key)

# Define the JSON schema for structured output
json_schema = {
    "name": "robot_task_instructions",  # Schema name
    "schema": {
        "type": "object",
        "properties": {
            "step": {
                "type": "integer",
                "description": "The current step number of the task."
            },
            "locations": {
                "type": "array",
                "items": {
                    "type": "string",
                    "enum": ["C", "S", "B", "T"],
                    "description": "The current location of each robot: Charging area (C), Storage area (S), Building area (B), or Traveling (T)."
                }
            },
            "actions": {
                "type": "array",
                "items": {
                    "type": "string",
                    "enum": ["MOVE_S", "MOVE_B", "MOVE_C", "PICK_V", "PICK_H", "BUILD", "CHARGE"],
                    "description": "The action that each robot should perform: Move to Storage (MOVE_S), Move to Building (MOVE_B), Move to Charging (MOVE_C), Pick vertical beam (PICK_V), Pick horizontal beam (PICK_H), Build (BUILD), or Charge (CHARGE)."
                }
            },
            "internal_cargo": {
                "type": "array",
                "items": {
                    "type": "integer",
                    "description": "The amount of cargo (materials) each robot is currently carrying."
                }
            },
            "placed_bricks": {
                "type": "integer",
                "description": "The number of bricks placed so far in the building task."
            },
            "remaining_battery": {
                "type": "array",
                "items": {
                    "type": "integer",
                    "description": "The remaining battery percentage for each robot."
                }
            }
        },
        "required": [
            "step",
            "locations",
            "actions",
            "internal_cargo",
            "placed_bricks",
            "remaining_battery"
        ],
        "additionalProperties": False
    },
    "strict": True
}


# Create the assistant with structured output using the function
assistant = client.beta.assistants.create(
    name="LLM_task_manager_JSON",
    instructions="""
    **Background information**

    Imagine you are a task scheduler agent. You need to plan a series of tasks involving 2 number of robots to fulfill a common goal. 
    There are three different areas in a hypothetical scenario (i.e., charging area, storage area, building area) that are equally separated.
    The robots can move freely between the three areas.
    The robots need to build a wall with 4 vertical beams and 2 horizontal beams. 
    All the needed beams are located in the storage area.
    Make sure to build the wall in a logical manner, that is to pick the correct beam based on how much has been built previously.
    A robot can only carry one beam at a time, so based on the battery level you might have to head back to charge before picking up another beam.
    The robots start with a fully charged battery at the charging area.
    Each robot consumes 20 percentage of battery per action. 
    ALL actions consume battery, except for charge and idle.
    The robots are able to perform one single action at a time (both robots can perform their action within the same time).  
    You need to keep an eye on the battery level of the robots, making sure that the robots have enough battery to go charge, if needed.
    The robots can ONLY charge in the charging area.
    You CANNOT charge anywhere else.
    You MUST ensure that the robots have enough battery to head to the charging area, otherwise the robot will run out of battery en-route.
    To charge, you MUST first go to the charging area, then issue the command to charge.
    Under no circumstances can the robots charge in the storage or building areas. Charging in any other area is an invalid action and must not be performed.
    When carrying out an action you need to make sure that the robot is at the right place, or the robot will not carry out the action.
    The available actions are:

    - Move (requires 1 time unit per distance unit)
    - Pick 1 unit of beam
    - Build 1 unit of beam
    - Charge (the battery gets fully recharged when commanded to charge at the charging area)
  

    The robots have the following characteristics:

    - Cargo capacity of 1 beam.
    - Capacity to build 1 beam per time unit. 

    Before providing the instructions, you will be provided with feedback regarding the current state of the robots. 
    Please respond with only the requested format, no introductory or explanatory text.
    Respond ONLY after receiving the feedback. 

    **API information**

    The type of feedback you get will be in the following format:

    {
        "locations": ["CURRENT_LOCATION_robot_1", "CURRENT_LOCATION_robot_2"],
        "internal_cargo": ["INTERNAL_CARGO_robot_1", "INTERNAL_CARGO_robot_2"],
        "remaining_battery": ["REMAINING_BATTERY_robot_1", "REMAINING_BATTERY_robot_2"]
    }
    
    **Example Feedback**
    {
    "locations": ["C", "C"],
    "internal_cargo": [0, 0],
    "remaining_battery": [100, 100]
    }
    
	
    You will respond ONLY by providing the instructions according to the provided API, no explanation or added introduction. 
    Where:

	- Current location can be C (charging area), S (storage area), B (building area) or T (to indicate the robot is travelling)
	- Action being performed can be MOVE_S, MOVE_B, MOVE_C, PICK_H, PICK_V, BUILD, CHARGE
	- Internal cargo can be an integer number
	- Placed beams can be an integer number
	- Remaining battery ranges from 0 to 100
    You will provide a breakdown of the action to be taken based on the previously completed steps and the robots current state to complete the task. The API format is the following:

    ```json
    {
        "step": <step_number>,
        "locations": ["CURRENT_LOCATION_robot_1", "CURRENT_LOCATION_robot_N"],
        "actions": ["ACTION_TO_PERFORM_robot_1", "ACTION_TO_PERFORM_robot_N"],
        "internal_cargo": ["INTERNAL_CARGO_robot_1", "INTERNAL_CARGO_robot_N"],
        "placed_bricks": <placed_bricks>,
        "remaining_battery": ["REMAINING_BATTERY_robot_1", "REMAINING_BATTERY_robot_N"]
    }
    ```

    **Example JSON response**

    ```json
    [
        {
            "step": 1,
            "locations": ["C", "C"],
            "actions": ["MOVE_S", "MOVE_S"],
            "internal_cargo": [0, 0],
            "placed_bricks": 0,
            "remaining_battery": [80, 80]
        }
    ]
    ```

    """,
    model="gpt-4o-mini",
    response_format= { "type": "json_schema", "json_schema": json_schema}
)

# Store the assistant ID
assistant_id = assistant.id

# Create the thread to interact with the assistant
thread = client.beta.threads.create()

# Store the thread ID and assistant ID in a file
with open('thread_info.txt', 'w') as file:
    file.write(f"THREAD_ID={thread.id}\n")
    file.write(f"ASSISTANT_ID={assistant_id}\n")

print(f"Thread ID created: {thread.id}")
print(f"Assistant ID created: {assistant_id}")

