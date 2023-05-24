import rclpy
from rclpy.node import Node
from cordyceps_interfaces.srv import CustomAss
from cordyceps_interfaces.msg import RobotPose, RobotPaths, Task, Path
import numpy as np

class CustomAssembler(Node):

    def __init__(self):
        super().__init__('minimal_service')
        
        self.assembler_service = self.create_service(CustomAss, 'get_robot_vs_ref_pose', self.get_robot_vs_ref_pose_callback)
        
    def get_robot_vs_ref_pose_callback(self, request, response):
        bot_0_pose = RobotPose()
        bot_1_pose = RobotPose()
        bot_2_pose = RobotPose()
        bot_3_pose = RobotPose()

        bot_0_pose.x = request.diameter / 2
        bot_0_pose.y = 0.0

        bot_1_pose.x = 0.0
        bot_1_pose.y = request.diameter / 2

        bot_2_pose.x = -request.diameter / 2
        bot_2_pose.y = 0.0

        bot_3_pose.x = 0.0
        bot_3_pose.y = -request.diameter / 2

        response.vs_ref_pose.append(bot_0_pose)
        response.vs_ref_pose.append(bot_1_pose)
        response.vs_ref_pose.append(bot_2_pose)
        response.vs_ref_pose.append(bot_3_pose)
        return response
        

def main(args=None):
    rclpy.init(args=args)
    custom_assembler = CustomAssembler()
    rclpy.spin(custom_assembler)
    custom_assembler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()