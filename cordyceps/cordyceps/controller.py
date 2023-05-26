import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from cordyceps_interfaces.action import Controller

class ControllerActionServer(Node):
    def __init__(self):
        super().__init__('cordyceps_controller')
        self.action_server = ActionServer(self, Controller, 'controller', self.execute_callback)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        goal_handle.succeed()
        print(goal_handle.request.robot_paths)
        result = Controller.Result()
        result.done = True
        return result

def main(args=None):
    rclpy.init(args=args)
    controller_action_server = ControllerActionServer()
    rclpy.spin(controller_action_server)
    controller_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()