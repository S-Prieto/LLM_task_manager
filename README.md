# LLM_task_manager
Main nodes coordinating the LLM-driven task manager for construction applications

Before running this, please make sure to change the folder names from **LLM_task_manager** to **llm_task_manager**.

After installing pyrobosim as usual, and building the workspace using:

```
colcon build
```
Make a .env file with the *OPENAI_API_KEY* set.

Open a terminal, navigate to the **llm_task_manager** subfolder and run

```
python3 create_thread.py
```
Source the install.bash file in the workspace in a new terminal and run the pyrobosim simulation with the command

```
ros2 launch pyrobosim_ros task_allocation.launch.py world_file:=task_planner_world.yaml
```

Next, in a new sourced terminal, run the command parser with:
```
ros2 run llm_task_manager command_parser 
```

Then, in another terminal, source the workspace, and run:
```
export ROS2GPT_ENV_PATH=/path/to/your_workspace/src/llm_task_manager/llm_task_manager/.env
ros2 run llm_task_manager openai_bridge
```
This should allow the assistant to control the robot.
Please let me know if any problems occur.


