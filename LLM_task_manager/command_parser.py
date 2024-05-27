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
        self.publisher = self.create_publisher(
            TaskAction,
            'commanded_action',
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        step_, current_location_, action_, internal_cargo_, placed_bricks_, battery_ = self.parse_message(msg.data)
        self.get_logger().info(f'Parsed values - Step: {step_}, Current Location: {current_location_}, Action: {action_}, Internal Cargo: {internal_cargo_}, Placed Bricks: {placed_bricks_}, Battery: {battery_}')
        self.publish_commanded_action(step_, current_location_, action_, internal_cargo_, placed_bricks_, battery_)

    def parse_message(self, message):
        pattern = r'\{STEP (\d+), \[(.*?)\], \[(.*?)\], \[(.*?)\], (\d+), \[(.*?)\]\}'
        match = re.match(pattern, message)
        if match:
            step_ = int(match.group(1))
            current_location_ = match.group(2)
            action_ = match.group(3)
            internal_cargo_ = match.group(4)
            placed_bricks_ = int(match.group(5))
            battery_ = match.group(6)
            return step_, current_location_, action_, internal_cargo_, placed_bricks_, battery_
        else:
            self.get_logger().error('Failed to parse message')
            return None, None, None, None, None, None
        
    def publish_commanded_action(self, step_, current_location_, action_, internal_cargo_, placed_bricks_, battery_):
        msg = TaskAction()
        msg.robot = 'robot_1'
        if action_.startswith("MOVE_"):
            msg.type = 'navigate'
            if action_ == "MOVE_C":
                msg.target_location = "charging"
            elif action_ == "MOVE_S":
                msg.target_location = "storage"
            elif action_ == "MOVE_B":
                msg.target_location = "build"
            else:
                self.get_logger().error(f'Unknown MOVE action: {action_}')
                return
        elif action_ == "CHARGE":
            msg.type = 'charge'
            msg.target_location = ""

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
