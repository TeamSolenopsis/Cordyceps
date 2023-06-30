import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
import csv
import os
from cordyceps_interfaces.srv import CustomPathPlanner
from cordyceps_interfaces.msg import Path, RobotRoutes, RobotPose


"""ROS2 Node that generates paths for each robot in the virtual structure"""
class PathPlanner(Node):
    def __init__(self):
        """Initializes the node and creates the service"""

        super().__init__('path_planner_service')
        self.path_planner_service = self.create_service(CustomPathPlanner, 'get_robot_routes', self.get_routes_callback)

        self.RESOLUTION = 10 # The amount of points in which the routes will be split.
        self.MAX_SPEED = 0.2 # Maximum allowed speed from a robot.(m/s)

        self.angle = 0.0  # rad

    def generate_vs_path_mock(self, start_pose:Pose) -> np.array:
        """Generates a path for the virtual structure to follow."""

        file_name = "8_robots_from_2-2.csv"
        file_dir = os.path.dirname(os.path.realpath('__file__'))
        file_path = os.path.join(file_dir, "src/Cordyceps/cordyceps/resource/", file_name)
        
        file = open(file_path,'r')

        data = list(csv.reader(file, delimiter=','))
        file.close()
        
        for i in range(len(data)):
            data[i] = [float(j) for j in data[i]]
        return data
         
    def get_routes_callback(self, request, response):
        """Generates a path for a robot to follow in order to reach a goal pose.
    
        :param Request request: The request containing the starting pose of the robot.
        :return: The path that the robot should follow.
        """

        vs_ref_pose = request.vs_ref_pose
        start_pose = request.task.start_pose
    
        # TODO: Trigger nav2 to create a path for VS
        vs_path = self.generate_vs_path_mock(start_pose)

        routes = RobotRoutes()
        
        fleet_size = request.task.number_of_robots
        bot_routes = []
        for _ in range(fleet_size):
            bot_routes.append(Path())

        for pose in vs_path:
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
                bot_routes[bot_i].robot_poses.append(bot_pose)

        for bot_path in bot_routes:
            routes.routes.append(bot_path)

        response.robot_routes = routes
        return response        

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()