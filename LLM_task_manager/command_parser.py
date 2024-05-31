import rclpy
import re

from rclpy.node import Node
from std_msgs.msg import String
from pyrobosim_msgs.msg import TaskAction


class ScheduleCommandsSubscriber(Node):
    def __init__(self):
        super().__init__('schedule_commands_subscriber')
        self.subscription = self.create_subscription(
            String,
            'schedule_commands',
            self.listener_callback,
            10)
        self.robot_1_subscription = self.create_subscription(
            String,
            'robot_1/robot_state',
            self.listener_callback,
            10)
        self.robot_2_subscription = self.create_subscription(
            String,
            'robot_2/robot_state',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            TaskAction,
            'commanded_action',
            10)
        
        # Initialize counters for pick actions
        self.pick_h_counter = 0
        self.pick_v_counter = 0

        self.robot_states = {"robot_1": None, "robot_2": None}

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        parsed_data = self.parse_message(msg.data)
        if parsed_data:
            (step_, current_location_, action_, internal_cargo_, 
             placed_bricks_, battery_) = parsed_data
            self.get_logger().info(f'Parsed values - Step: {step_}, '
                                   f'Current Locations: {current_location_}, '
                                   f'Actions: {action_}, Internal Cargos: {internal_cargo_}, '
                                   f'Placed Bricks: {placed_bricks_}, Batteries: {battery_}')
            for i, (current_location, action, cargo, battery) in enumerate(zip(current_location_, action_, internal_cargo_, battery_)):
                robot_id = f'robot_{i+1}'
                self.publish_commanded_action(robot_id, step_, current_location, action, cargo, placed_bricks_, battery)

    def robot_state_callback(self, msg):
        robot_id = msg._topic_name.split('/')[0]
        self.robot_states[robot_id] = msg.data


    def parse_message(self, message):
        pattern = r'\{STEP (\d+), \[(.*?)\], \[(.*?)\], \[(.*?)\], (\d+), \[(.*?)\]\}'
        match = re.match(pattern, message)
        if match:
            step_ = int(match.group(1))
            current_location_ = [loc.strip() for loc in match.group(2).split(',')]
            action_ = [act.strip() for act in match.group(3).split(',')]
            internal_cargo_ = [cargo.strip() for cargo in match.group(4).split(',')]
            placed_bricks_ = int(match.group(5))
            battery_ = [bat.strip() for bat in match.group(6).split(',')]
            return step_, current_location_, action_, internal_cargo_, placed_bricks_, battery_
        else:
            self.get_logger().error('Failed to parse message')
            return None
                
    def publish_commanded_action(self, robot_id, step_, current_location_, action_, internal_cargo_, placed_bricks_, battery_):
        msg = TaskAction()
        msg.robot = robot_id
        if action_.startswith("MOVE_"):
            msg.type = 'navigate'
            if action_ == "MOVE_C":
                msg.target_location = "charging"
            elif action_ == "MOVE_S":
                msg.target_location = "pallet0"
            elif action_ == "MOVE_B":
                msg.target_location = "pallet1"
            else:
                self.get_logger().error(f'Unknown MOVE action: {action_}')
                return
        elif action_ == "CHARGE":
            msg.type = 'charge'
            msg.target_location = ""
        elif action_ == "PICK_H":
            msg.type = 'pick'
            msg.object = f'h_beam{self.pick_h_counter}'
            self.pick_h_counter += 1
        elif action_ == 'PICK_V':
            msg.type = 'pick'
            msg.object = f'v_beam{self.pick_v_counter}'
            self.pick_v_counter += 1
        elif action_ == 'PLACE':
            msg.type = 'place'
            msg.target_location = 'pallet1'
            if robot_id in self.robot_states and self.robot_states[robot_id]:
                manipulated_object = self.robot_states[robot_id]
                if manipulated_object.startswith('h_beam'):
                    self.get_logger().info(f'Placing h_beam: {manipulated_object}')
                elif manipulated_object.startswith('v_beam'):
                    self.get_logger().info(f'Placing v_beam: {manipulated_object}')
                else:
                    self.get_logger().error(f'Unknown manipulated object: {manipulated_object}')
                    return
            else:
                self.get_logger().error(f'No robot state available for {robot_id}')
                return
            # msg.pose.position.x = -0.5
            # msg.pose.position.y = 4.7
            # msg.pose.orientation.x = 0.0
            # msg.pose.orientation.y = 0.0
            # msg.pose.orientation.z = 0.0 
            # msg.pose.orientation.w = 1.0
           
        else:
            self.get_logger().error(f'Unsupported action type: {action_}')
            return
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published commanded action: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = ScheduleCommandsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
