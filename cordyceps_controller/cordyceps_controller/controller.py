import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import Robot

class Controller(Node):
    def __init__(self):
        super().__init__('cordyceps_controller')
        

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription([Pose], '/path', self.path_callback, 10)

        self.create_publisher(Twist, 'r1/cmd_vel', 10)
        self.create_publisher(Twist, 'r2/cmd_vel', 10)
        self.create_publisher(Twist, 'r3/cmd_vel', 10)
        self.create_publisher(Twist, 'r4/cmd_vel', 10),
        self.robots = {Robot('r1', 0, 0), Robot('r2', 100, 0), Robot('r3', 100, 100), Robot('r4', 0, 100), }

def cmd_vel_callback(self, msg):
    #TODO: Implement dividing the cmd_vel into 4 cmd_vel for each robot 
    pass

def path_callback(self, msg):
    #TODO: Implement dividing the pose into 4 pose for each robot
    robot_path_dict = generate_paths(msg)
    pass

def generate_paths(self, poses) -> dict:
    for i in len(poses):
        for robot in self.robots:
            robot.cmd_vel_publisher.publish(calculate_cmd_vel_robot(robot, poses[i], poses[i+1]));   
            #TODO: base the robot pose on the robot's offset from the center
            pass

    return {}


def calculate_cmd_vel_robot(self, robot, current_pose, goal_pose) -> Twist:
    #TODO: Calculate the cmd_vel for the robot based on the current pose and the goal pose
    return Twist()


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# def compute_trans_matrices(self):
#     self.trans_matrices = []
#     for r_pose in self.robot_poses:
#         theta = r_pose[2]

#         cos = np.cos(theta)
#         sin = np.sin(theta)

#         forward_r_trans = np.array(
#             ((cos, -sin, r_pose[0]), (sin, cos, r_pose[1]), (0, 0, 1))
#         )
#         inverse_r_trans = np.linalg.inv(forward_r_trans)
#         self.trans_matrices.append(inverse_r_trans)

# def compute_inv_kin(self, pose_goal):
#     pose_delta = pose_goal - self.pose
#     self.compute_trans_matrices()

#     return [np.dot(tf, pose_delta) for tf in self.trans_matrices]