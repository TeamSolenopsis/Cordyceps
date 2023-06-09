import rclpy
from rclpy.node import Node
from cordyceps_interfaces.srv import CustomRobotAssembler, AssemblerGetVsRefPose
from cordyceps_interfaces.msg import RobotPose, RobotPaths, Task, Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Pose
import numpy as np
import math

class Assembler(Node):

    def __init__(self):
        super().__init__('assembler')
        
        self.assembler_service = self.create_service(AssemblerGetVsRefPose, 'get_robot_vs_ref_pose', self.get_robot_vs_ref_pose_callback)
        self.assembler_service = self.create_service(CustomRobotAssembler, 'assemble_robots', self.assemble_robots_callback)
        
    def get_robot_vs_ref_pose_callback(self, request, response):
        task = request.task
        bot_pose = RobotPose()
        navigator = BasicNavigator()

        for robot_index in range(task.number_of_robots):
            angle = (2 * np.pi * robot_index) / task.number_of_robots
            bot_pose.x = float(np.cos(angle) * task.diameter / 2)
            bot_pose.y = float(np.sin(angle) * task.diameter / 2)
            print(f'robot:{robot_index} x: {bot_pose.x}, y: {bot_pose.y}') 
            response.vs_ref_pose.append(bot_pose)

        return response

    #TODO: test this function 
    #TODO: test the way robot_poses is an np.array
    def assemble_robots_callback(self, request, response):
        task = request.task
        robot_poses = np.array()
        robot_poses = self.get_robot_vs_ref_pose_callback(self, request, response)
        print(robot_poses)

        w, x, y, z = task.start_pose.pose.pose.orientation.w, task.start_pose.pose.pose.orientation.x, task.start_pose.pose.pose.orientation.y, task.start_pose.pose.pose.orientation.z
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = round(math.atan2(siny_cosp, cosy_cosp),2)

        tf_matrix = np.array(
                [
                    [np.cos(yaw), -np.sin(yaw), task.start_pose.pose.pose.point.x],
                    [np.sin(yaw), np.cos(yaw), task.start_pose.pose.pose.point.y],
                    [0, 0, 1],
                ]
            ) 

        for robot_index in range(task.number_of_robots):
            robot_start_position = Pose()

            robot_start_position = np.matmul(tf_matrix, robot_poses[robot_index])

            #TODO: send to MQTT with robot id


            #TODO: wait till all robots have sent on position message

        return response
        
def main(args=None):
    rclpy.init(args=args)
    assembler = Assembler()
    rclpy.spin(assembler)
    assembler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()