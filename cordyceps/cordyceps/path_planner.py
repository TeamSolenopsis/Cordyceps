import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
import math
import csv
from cordyceps_interfaces.srv import CustomPathPlanner, CustomRobotAssembler
from cordyceps_interfaces.msg import Path, RobotPaths, RobotPose, Task


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_service')
        self.path_planner_service = self.create_service(CustomPathPlanner, 'get_robot_paths', self.get_robot_paths_callback)

        self.RESOLUTION = 10 # The amount of points in which the paths will be split.
        self.MAX_SPEED = 0.5 # Maximum allowed speed from a robot.(m/s)

        self.angle = 0.0  # rad

    def generate_vs_path_mock(self, start_pose:Pose) -> np.array:
        file = open('src/Cordyceps/cordyceps/cordyceps/Path_circle.csv','r')
        data = list(csv.reader(file, delimiter=','))
        file.close()
        
        for i in range(len(data)):
            data[i] = [float(j) for j in data[i]]
        return data
         
    def get_robot_paths_callback(self, request, response):
        vs_ref_pose = request.vs_ref_pose
        start_pose = request.task.start_pose
    
        # TODO: Trigger nav2 to create a path for VS
        vs_path = self.generate_vs_path_mock(start_pose)

        robot_paths = RobotPaths()
        bot_0_path = Path()
        bot_1_path = Path()
        bot_2_path = Path()
        bot_3_path = Path()

        for pose in vs_path:
            # transformation matrix template.   
            tf_matrix = np.array(
                [
                    [np.cos(pose[2]), -np.sin(pose[2]), pose[0]],
                    [np.sin(pose[2]), np.cos(pose[2]), pose[1]],
                    [0, 0, 1],
                ]
            )           

        # Robot coordinates. 
            bot_0_xy = np.array([vs_ref_pose[0].x, vs_ref_pose[0].y, 1])
            bot_1_xy = np.array([vs_ref_pose[1].x, vs_ref_pose[1].y, 1]) 
            bot_2_xy = np.array([vs_ref_pose[2].x, vs_ref_pose[2].y, 1])
            bot_3_xy = np.array([vs_ref_pose[3].x, vs_ref_pose[3].y, 1])


            # Calculation for pose of every robot in the VS.
            trans_0 = np.matmul(tf_matrix, bot_0_xy)
            trans_1 = np.matmul(tf_matrix, bot_1_xy)
            trans_2 = np.matmul(tf_matrix, bot_2_xy)
            trans_3 = np.matmul(tf_matrix, bot_3_xy)

            # Calculated path for every robot in the VS.

            bot_0_pose = RobotPose()
            bot_1_pose = RobotPose()
            bot_2_pose = RobotPose()
            bot_3_pose = RobotPose()

            bot_0_pose.x = trans_0[0]
            bot_1_pose.x = trans_1[0]
            bot_2_pose.x = trans_2[0]
            bot_3_pose.x = trans_3[0]

            bot_0_pose.y = trans_0[1]
            bot_1_pose.y = trans_1[1]
            bot_2_pose.y = trans_2[1]
            bot_3_pose.y = trans_3[1]

            bot_0_path.robot_poses.append(bot_0_pose)
            bot_1_path.robot_poses.append(bot_1_pose)
            bot_2_path.robot_poses.append(bot_2_pose)
            bot_3_path.robot_poses.append(bot_3_pose)

        robot_paths.paths.append(bot_0_path)
        robot_paths.paths.append(bot_1_path)
        robot_paths.paths.append(bot_2_path)
        robot_paths.paths.append(bot_3_path)

        response.robot_paths = robot_paths
        return response        

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()