import rclpy
from rclpy.node import Node
from cordyceps_interfaces.srv import CustomRobotAssembler
from cordyceps_interfaces.msg import RobotPose
from .Robot import Robot
import numpy as np

class Assembler(Node):
    
    def __init__(self):
        """Constructor for the Assembler class. Initializes the ROS2 node and creates the service."""
        super().__init__('assembler')
        
        self.assembler_service = self.create_service(CustomRobotAssembler, 'get_robot_vs_ref_pose', self.get_robot_vs_ref_pose_callback)

        self.robots = []
     
    def get_robot_vs_ref_pose_callback(self, request, response):
        """Callback function for the assembler service.
        
        :param request: Request message for the service.
        :param response: Response message for the service.
        
        :returns: The response message with the reference poses for each robot."""

        task = request.task

        # for robot_index in range(task.number_of_robots):
        #     # angle = (2 * np.pi * robot_index) / task.number_of_robots
        #     # bot_pose.x = float(np.cos(angle) * task.diameter / 2)
        #     # bot_pose.y = float(np.sin(angle) * task.diameter / 2)
            # print(f'robot:{robot_index} x: {bot_pose.x}, y: {bot_pose.y}') 
            # response.vs_ref_pose.append(bot_pose)


        bot_pose_0 = RobotPose()
        bot_pose_0.x = 0.20
        bot_pose_0.y = 0.0

        bot_pose_1 = RobotPose()
        bot_pose_1.x= -0.22
        bot_pose_1.y = -0.18

        bot_pose_2 = RobotPose()
        bot_pose_2.x = -0.22
        bot_pose_2.y = 0.18            

        response.vs_ref_pose.append(bot_pose_0)
        response.vs_ref_pose.append(bot_pose_1)
        response.vs_ref_pose.append(bot_pose_2)


        for robot_index in range(task.number_of_robots):
            self.add_robot(task.number_of_robots, robot_index)

        return response
    
    def add_robot(self, number_of_robots, robot_number):
        if len(self.robots) < number_of_robots:
            robot = Robot(0,0,0,f"r{robot_number}", self)
            self.robots.append(robot)
        

def main(args=None):
    rclpy.init(args=args)
    assembler = Assembler()
    rclpy.spin(assembler)
    assembler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()