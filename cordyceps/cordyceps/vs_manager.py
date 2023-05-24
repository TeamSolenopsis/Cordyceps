import rclpy
from rclpy.node import Node
from rclpy.node import ActionClient

# from cordyceps_interfaces.msg import Arrived
# from cordyceps_interfaces.msg import PoseShape
# from cordyceps_interfaces.msg import Assembled  

from cordyceps_interfaces.action import Controller
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import time
import string
import matplotlib.pyplot as plt



# from cordyceps_interfaces.msg import arrived
# from cordyceps_interfaces.msg import pose_shape
# from cordyceps_interfaces.msg import assembled  
class Vs_manager(Node):

    def __init__(self):
        super().__init__('vs_manager')
        self.controller_action_client = ActionClient(self, Controller, 'controller')

    def send_goal(self, order):
        goal_msg = Controller.Goal()
        goal_msg.order = order
        self.controller_action_client.wait_for_server()
        self.controller_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.m2p = 3779

        self.RESOLUTION = 1000 # The amount of points in which the paths will be split.
        self.MAX_SPEED = 0.5 # Maximum allowed speed from a robot.(m/s)

        
        self.vs_origin_x = 400 / self.m2p  # pixels
        self.vs_origin_y = 400 / self.m2p  # pixels

        self.x = np.linspace(1.0, 5, self.RESOLUTION)
        self.y_lins = np.linspace(1.0, 5, self.RESOLUTION)
        self.y = (np.sin(self.y_lins) * 7)     

        self.angle = 0.0  # rad

        self.distance_to_vs_pixels = 50   # pixels
        self.distance_to_vs = self.distance_to_vs_pixels / self.m2p   # meters

        self.robot_paths = self.generate_paths(list(zip(self.x, self.y)))      
        self.plot_path(self.robot_paths, True)


    def generate_paths(self, vs_path) -> dict:
        path = []

        for  pose, next_pose in zip(vs_path, vs_path[1:]):

            x_goal = (next_pose[0] - pose[0])
            y_goal = (next_pose[1] - pose[1])

            angle = np.arctan(y_goal / x_goal)
            
            if y_goal < 0 and x_goal < 0:
                    angle += np.pi
            if y_goal > 0 and x_goal < 0:
                    angle -= np.pi

            # if y_goal < 0:  # check if the angle is above pi.
            #    angle += np.pi

            # transformation matrix template.   
            tf_matrix = np.array(
                [
                    [np.cos(angle), -np.sin(angle), pose[0]],
                    [np.sin(angle), np.cos(angle), pose[1]],
                    [0, 0, 1],
                ]
            )  

            # Robot coordinates.
            bot_0_xy = np.array([self.distance_to_vs, -self.distance_to_vs, 1])
            bot_1_xy = np.array([self.distance_to_vs, self.distance_to_vs, 1])
            bot_2_xy = np.array([-self.distance_to_vs, self.distance_to_vs, 1])
            bot_3_xy = np.array([-self.distance_to_vs, -self.distance_to_vs, 1])

            # Calculation for pose of every robot in the VS.
            trans_0 = tf_matrix.dot(bot_0_xy)
            trans_1 = tf_matrix.dot(bot_1_xy)
            trans_2 = tf_matrix.dot(bot_2_xy)
            trans_3 = tf_matrix.dot(bot_3_xy)

            # Calculated path for every robot in the VS.
            path.append(
                (
                    ([trans_0[0] + self.vs_origin_x, trans_0[1] + self.vs_origin_y, angle]),
                    ([trans_1[0] + self.vs_origin_x, trans_1[1] + self.vs_origin_y, angle]),
                    ([trans_2[0] + self.vs_origin_x, trans_2[1] + self.vs_origin_y, angle]),
                    ([trans_3[0] + self.vs_origin_x, trans_3[1] + self.vs_origin_y, angle]),
                )
            )

        for vs_pose_id, vs_pose in enumerate(path):
            for bot_pose_id, bot_pose in enumerate(vs_pose):
                try:
                    yy = path[vs_pose_id + 1][bot_pose_id][1] - bot_pose[1]
                    xx = path[vs_pose_id + 1][bot_pose_id][0] - bot_pose[0]
                except:
                    alpha=alpha
                alpha = -np.arctan(yy / xx)

                # if yy < 0 and xx < 0:
                #     alpha += np.pi
                # if yy > 0 and xx < 0:
                #     alpha -= np.pi

                bot_pose[2] = alpha  

        return path        



    def plot_path(self, path, show:bool):
        li_r1 = []
        li_r2 = []
        li_r3 = []
        li_r4 = []
        for i, pose in enumerate(path):
            li_r1.append([pose[0][0], pose[0][1]])
            li_r2.append([pose[1][0], pose[1][1]])
            li_r3.append([pose[2][0], pose[2][1]])
            li_r4.append([pose[3][0], pose[3][1]])
            
        plt.scatter(*zip(*li_r1), s=3)
        plt.scatter(*zip(*li_r2), s=3)
        plt.scatter(*zip(*li_r3), s=3)
        plt.scatter(*zip(*li_r4), s=3)

        if show:       
            plt.show()


def main(args=None):
    rclpy.init(args=args)
    vs_manager = Vs_manager()
    rclpy.spin(vs_manager)
    vs_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()