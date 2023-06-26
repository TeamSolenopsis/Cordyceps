import rclpy
from rclpy.node import Node
from cordyceps_interfaces.srv import CustomRobotAssembler
from cordyceps_interfaces.msg import RobotPose, RobotRoutes, Task, Path
import numpy as np

class Assembler(Node):
    
    def __init__(self):
        """Constructor for the Assembler class. Initializes the ROS2 node and creates the service."""
        super().__init__('assembler')
        
        self.assembler_service = self.create_service(CustomRobotAssembler, 'get_robot_vs_ref_pose', self.get_robot_vs_ref_pose_callback)
     
    def get_robot_vs_ref_pose_callback(self, request, response):
        """Callback function for the assembler service.
        
        :param request: Request message for the service.
        :param response: Response message for the service.
        
        :returns: The response message with the reference poses for each robot."""

        task = request.task
        bot_0_pose = RobotPose()
        bot_1_pose = RobotPose()
        bot_2_pose = RobotPose()
        bot_3_pose = RobotPose()

        bot_0_pose.x = task.diameter /2
        bot_0_pose.y = 0.0

        bot_1_pose.x = 0.0
        bot_1_pose.y = task.diameter /2

        bot_2_pose.x = -task.diameter /2
        bot_2_pose.y = 0.0

        bot_3_pose.x = 0.0
        bot_3_pose.y = -task.diameter /2

        response.vs_ref_pose.append(bot_0_pose)
        response.vs_ref_pose.append(bot_1_pose)
        response.vs_ref_pose.append(bot_2_pose)
        response.vs_ref_pose.append(bot_3_pose)
        return response
        

def main(args=None):
    rclpy.init(args=args)
    assembler = Assembler()
    rclpy.spin(assembler)
    assembler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()