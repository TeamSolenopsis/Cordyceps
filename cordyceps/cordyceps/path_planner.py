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

        file = open('/home/sara/Documents/Fontys_Minor/ros_ws/src/Cordyceps/cordyceps/resource/Path1.csv','r')
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
        
        fleet_size = request.task.number_of_robots
        bot_paths = []
        for _ in range(fleet_size):
            bot_paths.append(Path())


        for pose in vs_path:
            # transformation matrix template.   
            tf_matrix = np.array(
                [
                    [np.cos(pose[2]), -np.sin(pose[2]), pose[0]],
                    [np.sin(pose[2]), np.cos(pose[2]), pose[1]],
                    [0, 0, 1],
                ]
            )     

            for bot_i in range(fleet_size): 
                bot_xy = np.array([vs_ref_pose[bot_i].x, vs_ref_pose[bot_i].y, 1])
                trans = np.matmul(tf_matrix, bot_xy)
                bot_pose = RobotPose()
                bot_pose.x = trans[0]
                bot_pose.y = trans[1]
                bot_paths[bot_i].robot_poses.append(bot_pose)

        for bot_path in bot_paths:
            robot_paths.paths.append(bot_path)

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