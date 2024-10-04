import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from pyrobosim_msgs.msg import TaskAction
from pyrobosim_msgs.msg import RobotState

class ScheduleCommandsSubscriber(Node):
    def __init__(self):
        super().__init__('schedule_commands_subscriber')
        self.subscription = self.create_subscription(
            String,
            'schedule_commands',
            self.listener_callback,
            10)
        self.robot_1_subscription = self.create_subscription(
            RobotState,
            'robot_1/robot_state',
            self.robot_state_callback,
            10)
        self.robot_2_subscription = self.create_subscription(
            RobotState,
            'robot_2/robot_state',
            self.robot_state_callback,
            10)
        self.publisher = self.create_publisher(
            TaskAction,
            'commanded_action',
            10)

        # Initialize counters for pick and place actions
        self.pick_h_counter = 0
        self.pick_v_counter = 0
        self.place_h_counter = 0
        self.place_v_counter = 0

        # Track robot states and current actions
        self.robot_states = {"robot_1": None, "robot_2": None}
        self.current_actions = {"robot_1": None, "robot_2": None}  # Track current actions
        self.robot_locations = {"robot_1": None, "robot_2": None}  # Track robot locations

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')

        try:
            # Parse the incoming message as JSON
            command_data = json.loads(msg.data)

            # Log the entire command data
            self.get_logger().info(f'Command Data: {command_data}')

            # Extract command details
            step = command_data.get('step')
            locations = command_data.get('locations', [])
            actions = command_data.get('actions', [])
            internal_cargo = command_data.get('internal_cargo', [])
            placed_bricks = command_data.get('placed_bricks', 0)
            remaining_battery = command_data.get('remaining_battery', [])

            self.get_logger().info(f'Parsed Step {step}:')
            self.get_logger().info(f'  Current Locations: {locations}')
            self.get_logger().info(f'  Actions: {actions}')
            self.get_logger().info(f'  Internal Cargo: {internal_cargo}')
            self.get_logger().info(f'  Placed Bricks: {placed_bricks}')
            self.get_logger().info(f'  Remaining Battery: {remaining_battery}')

            # Dispatch actions for each robot
            for i, (location, action, cargo, battery) in enumerate(zip(locations, actions, internal_cargo, remaining_battery)):
                robot_id = f'robot_{i+1}'
                
                # Check if the robot is already performing the same action
                if self.current_actions[robot_id] == action:
                    self.get_logger().info(f'{robot_id} is already performing action "{action}". Ignoring duplicate command.')
                    continue  # Skip this command since it's a duplicate

                self.publish_commanded_action(robot_id, step, location, action, cargo, placed_bricks, battery)

        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse the message as JSON')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')

    def robot_state_callback(self, msg):
        robot_id = msg.name
        self.robot_states[robot_id] = msg.manipulated_object
        self.robot_locations[robot_id] = msg.current_location  # Track current location

        # Reset current action if the robot has completed its task
        if msg.manipulated_object is None:  # Assuming None means the task is complete
            self.current_actions[robot_id] = None

    def publish_commanded_action(self, robot_id, step, location, action, cargo, placed_bricks, battery):
        msg = TaskAction()
        msg.robot = robot_id
        current_object = self.robot_states.get(robot_id)
        current_location = self.robot_locations.get(robot_id)  # Get current location

        if action.startswith("MOVE_"):
            msg.type = 'navigate'
            if action == "MOVE_C":
                msg.target_location = "charging"
            elif action == "MOVE_S":
                msg.target_location = "pallet0"
            elif action == "MOVE_B":
                msg.target_location = "pallet1"
            else:
                self.get_logger().error(f'Unknown MOVE action: {action}')
                return
        elif action == "CHARGE":
            # Ensure the robot is in the charging location before allowing it to charge
            if current_location != "charging":
                self.get_logger().error(f'{robot_id} cannot charge because it is not in the charging area. Current location: {current_location}')
                return
            msg.type = 'charge'
            msg.target_location = ""
        elif action == "PICK_H":
            if current_object:
                self.get_logger().error(f'{robot_id} cannot pick up beams while holding {current_object}')
                return
            # Increment counter only if pick action is successful
            if self.pick_beam(robot_id, 'h_beam', self.pick_h_counter, msg):
                self.pick_h_counter += 1
            else:
                self.get_logger().error(f'Failed to pick h_beam for {robot_id}')
                return
        elif action == 'PICK_V':
            if current_object:
                self.get_logger().error(f'{robot_id} cannot pick beams while holding {current_object}')
                return
            # Increment counter only if pick action is successful
            if self.pick_beam(robot_id, 'v_beam', self.pick_v_counter, msg):
                self.pick_v_counter += 1
            else:
                self.get_logger().error(f'Failed to pick v_beam for {robot_id}')
                return
        elif action == 'BUILD':
            if not current_object:
                self.get_logger().error(f'{robot_id} is not holding any object to place.')
                return
            if not self.place_beam(robot_id, msg):
                self.get_logger().error(f'Failed to place beam for {robot_id}')
                return
        else:
            self.get_logger().error(f'Unsupported action type: {action}')
            return

        self.publisher.publish(msg)
        self.get_logger().info(f'Published commanded action: {msg}')

        # Update the current action being executed by the robot
        self.current_actions[robot_id] = action

    def pick_beam(self, robot_id, beam_type, counter, msg):
        msg.type = 'pick'
        msg.object = f'{beam_type}{counter}'
        if msg.object in self.robot_states.values():
            self.get_logger().error(f'{robot_id} failed to pick {beam_type}{counter}, it is already being manipulated by another robot.')
            return False
        return True

    def place_beam(self, robot_id, msg):
        msg.type = 'place'
        msg.target_location = 'pallet1'

        if robot_id in self.robot_states and self.robot_states[robot_id]:
            manipulated_object = self.robot_states[robot_id]
            if manipulated_object.startswith('h_beam'):
                if self.place_h_counter == 0:
                    self.get_logger().info(f'Placing h_beam bottom: {manipulated_object}')
                    msg.pose.position.x = 0.0
                    msg.pose.position.y = 4.2
                    self.place_h_counter += 1
                elif self.place_h_counter == 1:
                    self.get_logger().info(f'Placing h_beam top: {manipulated_object}')
                    msg.pose.position.x = 0.0
                    msg.pose.position.y = 4.8

                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.7071068
                msg.pose.orientation.w = 0.7071068
                msg.has_pose = True

            elif manipulated_object.startswith('v_beam'):
                if self.place_v_counter == 0:
                    self.get_logger().info(f'Placing v_beam_0: {manipulated_object}')
                    msg.pose.position.x = -0.5
                    self.place_v_counter += 1
                elif self.place_v_counter == 1:
                    self.get_logger().info(f'Placing v_beam_1: {manipulated_object}')
                    msg.pose.position.x = -0.134
                    self.place_v_counter += 1
                elif self.place_v_counter == 2:
                    self.get_logger().info(f'Placing v_beam_2: {manipulated_object}')
                    msg.pose.position.x = 0.23
                    self.place_v_counter += 1
                elif self.place_v_counter == 3:
                    self.get_logger().info(f'Placing v_beam_3: {manipulated_object}')
                    msg.pose.position.x = 0.6
                    self.place_v_counter += 1

                msg.pose.position.y = 4.5
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.0
                msg.pose.orientation.w = 1.0
                msg.has_pose = True

            else:
                self.get_logger().error(f'Unknown manipulated object: {manipulated_object}')
                return False
        else:
            self.get_logger().error(f'No robot state available for {robot_id}')
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = ScheduleCommandsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

