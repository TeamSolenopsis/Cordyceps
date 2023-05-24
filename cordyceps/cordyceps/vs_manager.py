import rclpy
from rclpy.node import Node
from rclpy.node import ActionClient

# from cordyceps_interfaces.msg import Arrived
# from cordyceps_interfaces.msg import PoseShape
# from cordyceps_interfaces.msg import Assembled  

from cordyceps_interfaces.action import Controller

class Vs_manager(Node):

    def __init__(self):
        super().__init__('vs_manager')
        self.controller_action_client = ActionClient(self, Controller, 'controller')

    def send_goal(self, order):
        goal_msg = Controller.Goal()
        goal_msg.order = order
        self.controller_action_client.wait_for_server()
        self.controller_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)



def main(args=None):
    rclpy.init(args=args)
    vs_manager = Vs_manager()
    rclpy.spin(vs_manager)
    vs_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()