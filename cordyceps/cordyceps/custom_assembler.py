import rclpy
from rclpy.node import Node
from cordyceps_interfaces.srv import CustomRobotAssembler
from cordyceps_interfaces.msg import RobotPose, RobotPaths, Task, Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import numpy as np

class Assembler(Node):

    def __init__(self):
        super().__init__('assembler')
        
        self.assembler_service = self.create_service(CustomRobotAssembler, 'get_robot_vs_ref_pose', self.get_robot_vs_ref_pose_callback)
        
    def get_robot_vs_ref_pose_callback(self, request, response):
        task = request.task
        bot_pose = RobotPose()
        navigator = BasicNavigator()

        for robot_index in range(4):
            angle = (2 * np.pi / 4) * robot_index
            bot_pose.x = int(0 + np.cos(angle) * task.diameter / 2)
            bot_pose.y = int(0 + np.sin(angle) * task.diameter / 2)
            print(f'robot:{robot_index} x: {bot_pose.x}, y: {bot_pose.y}') 
            response.vs_ref_pose.append(bot_pose)

        return response
        

def main(args=None):
    rclpy.init(args=args)
    assembler = Assembler()
    rclpy.spin(assembler)
    assembler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()