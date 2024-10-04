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
            "placed_beams": {
                "type": "integer",
                "description": "The number of beams placed so far in the building task."
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
            "placed_beams",
            "remaining_battery"
        ],
        "additionalProperties": False
    },
    "strict": True
}


# Create the assistant with structured output using the function
instruction_generator = client.beta.assistants.create(
    name="Instruction_Generator",
    instructions="""
**Background information**

Imagine you are a task scheduler agent. You need to plan a series of tasks involving 2 robots to fulfill a common goal. 
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
You will then be given further feedback based on the instructions on how to improve them.
Please respond with only the requested format, no introductory or explanatory text.
Respond ONLY after receiving the feedback. 

**API information**

The state of the robot you get will be in the following format:

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

The further feedback will be in text form explaining what must be changed in the instructions.

You will respond ONLY by providing the instructions according to the provided API, no explanation or added introduction. 
Where:

- Current location can be C (charging area), S (storage area), B (building area) or T (to indicate the robot is travelling)
- Action being performed can be MOVE_S, MOVE_B, MOVE_C, PICK_H, PICK_V, BUILD, CHARGE
- Internal cargo can be an integer number
- Placed beams can be an integer number
- Remaining battery ranges from 0 to 100
You will provide a breakdown of the action to be taken based on the previously completed steps and the robots' current state to complete the task. The API format is the following:

```json
{
    "step": <step_number>,
    "locations": ["CURRENT_LOCATION_robot_1", "CURRENT_LOCATION_robot_N"],
    "actions": ["ACTION_TO_PERFORM_robot_1", "ACTION_TO_PERFORM_robot_N"],
    "internal_cargo": ["INTERNAL_CARGO_robot_1", "INTERNAL_CARGO_robot_N"],
    "placed_beams": <placed_beams>,
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
            "placed_beams": 0,
            "remaining_battery": [80, 80]
        }
    ]
    ```

    """,
    model="gpt-4o-mini",
    response_format= { "type": "json_schema", "json_schema": json_schema}
)

# Store the assistant ID
instruction_generator_id = instruction_generator.id

# Create the supervisor assistant

supervisor = client.beta.assistants.create(
    name="Supervisor",
    instructions="""You are a supervisor. Your task is to evaluate the instructions generated by the Instruction Generator assistant.
    The series of tasks involves 2 robots to fulfill a common goal. 
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
    The robots can move while carrying a beam, and this does not increase the battery consumption.
    The robots can also charge while carrying a beam, while in the chargin area.  
    You need to keep an eye on the battery level of the robots, making sure that the robots have enough battery to go charge, if needed.
    The robots can ONLY charge in the charging area.
    You CANNOT charge anywhere else.
    The BUILD task ensures that the robot places and builds the beam, and can only be carried out in the buildig area.
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

    The instruction generator will generate the instructions in the following format

    ```json
{
    "step": <step_number>,
    "locations": ["CURRENT_LOCATION_robot_1", "CURRENT_LOCATION_robot_N"],
    "actions": ["ACTION_TO_PERFORM_robot_1", "ACTION_TO_PERFORM_robot_N"],
    "internal_cargo": ["INTERNAL_CARGO_robot_1", "INTERNAL_CARGO_robot_N"],
    "placed_beams": <placed_beams>,
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
            "placed_beams": 0,
            "remaining_battery": [80, 80]
        }
    ]
    ```
    - Current location can be C (charging area), S (storage area), B (building area) or T (to indicate the robot is travelling)
    - Action being performed can be MOVE_S, MOVE_B, MOVE_C, PICK_H, PICK_V, BUILD, CHARGE
    - Internal cargo can be an integer number
    - Placed beams can be an integer number
    - Remaining battery ranges from 0 to 100
    
    **Evaluation Criteria**
    - Ensure the instructions make logical sense given the robot state.
    - Check that the instructions follow the rules of the instruction generation.
    - If the instructions are acceptable, respond with 'ACCEPTED'.
    - If not, provide clear feedback on what is wrong with the instructions
    
    Please respond only with 'ACCEPTED' or the feedback, no additional text. """,
    model="gpt-4o-mini",
    response_format={"type": "text"}
    )

supervisor_id = supervisor.id

# Create the thread to interact with the assistant
thread = client.beta.threads.create()

# Store the thread ID and assistant ID in a file
with open('thread_info.txt', 'w') as file:
    file.write(f"THREAD_ID={thread.id}\n")
    file.write(f"INSTRUCTION_GENERATOR_ID={instruction_generator.id}\n")
    file.write(f"SUPERVISOR_ID={supervisor_id}\n")
    

print(f"Thread ID created: {thread.id}")
print(f"Instruction Generator ID created: {instruction_generator.id}")
print(f"Supervisor Assistant ID created: {supervisor_id}")
