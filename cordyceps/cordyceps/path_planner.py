import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from cordyceps_interfaces.srv import CustomPathPlanner
from cordyceps_interfaces.msg import Path, RobotPaths, RobotPose, Task


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_service')
        self.path_planner_service = self.create_service(CustomPathPlanner, 'get_robot_paths', self.get_robot_paths_callback)

        self.RESOLUTION = 1000 # The amount of points in which the paths will be split.
        self.MAX_SPEED = 0.5 # Maximum allowed speed from a robot.(m/s)

        self.vs_origin_x = 0  # pixels
        self.vs_origin_y = 0  # pixels

        self.angle = 0.0  # rad

        self.distance_to_vs = 1   # meters

    def generate_vs_path(self, vs_origin_x, vs_origin_y) -> list:
        x = [vs_origin_x]
        y = [vs_origin_y]
        angles = []

        # right
        x = np.append(x, np.linspace(x[-1] - 1, x[-1], self.RESOLUTION))
        y = np.append(y, np.linspace(y[-1],y[-1], self.RESOLUTION))

        # turn right
        r = 4
        i = np.linspace(0*np.pi, 0.5*np.pi, self.RESOLUTION)
        x = np.append(x, (x[-1]) + np.flip(np.cos(i)*r))
        y = np.append(y, (y[-1]- r) + np.flip(np.sin(i)*r))

        for i in range(len(x) - 1):
            x_goal = (x[i + 1] - x[i])
            y_goal = (y[i + 1] - y[i])

            if x_goal != 0:
                angle = np.arctan(y_goal / x_goal)
            else:
                angle = 0
            
            if y_goal <= 0 and x_goal <= 0:
                    angle += np.pi
            if y_goal >= 0 and x_goal <= 0:
                    angle -= np.pi
            angles.append(angle)

        return list(zip(x, y, angles))
    
    def get_robot_paths_callback(self, request, response):
        # TODO: Call Assembler to get robot positions in relation to the vs.
    
        # Robot coordinates. 
        bot_0_xy = np.array([self.distance_to_vs, 0, 1])
        bot_1_xy = np.array([0, self.distance_to_vs, 1]) 
        bot_2_xy = np.array([-self.distance_to_vs, 0, 1])
        bot_3_xy = np.array([0, -self.distance_to_vs, 1])
        
        # TODO: Trigger nav2 to create a path for VS
        vs_path = self.generate_vs_path(self.vs_origin_x, self.vs_origin_y)

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

            # Calculation for pose of every robot in the VS.
            trans_0 = tf_matrix.dot(bot_0_xy)
            trans_1 = tf_matrix.dot(bot_1_xy)
            trans_2 = tf_matrix.dot(bot_2_xy)
            trans_3 = tf_matrix.dot(bot_3_xy)

            # Calculated path for every robot in the VS.

            bot_0_pose = RobotPose()
            bot_1_pose = RobotPose()
            bot_2_pose = RobotPose()
            bot_3_pose = RobotPose()

            bot_0_pose.x = trans_0[0] + self.vs_origin_x
            bot_1_pose.x = trans_1[0] + self.vs_origin_x
            bot_2_pose.x = trans_2[0] + self.vs_origin_x
            bot_3_pose.x = trans_3[0] + self.vs_origin_x

            bot_0_pose.y = trans_0[1] + self.vs_origin_y
            bot_1_pose.y = trans_1[1] + self.vs_origin_y
            bot_2_pose.y = trans_2[1] + self.vs_origin_y
            bot_3_pose.y = trans_3[1] + self.vs_origin_y

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