import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import time

class Controller(Node):
    def __init__(self):
        super().__init__('cordyceps_controller')
        
        # Outputs: cmd_vel for each robot
        self.cmd_vel_publisher = [
            self.create_publisher(Twist, 'r1/cmd_vel', 10),
            self.create_publisher(Twist, 'r2/cmd_vel', 10),
            self.create_publisher(Twist, 'r3/cmd_vel', 10),
            self.create_publisher(Twist, 'r4/cmd_vel', 10),
        ]  


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()