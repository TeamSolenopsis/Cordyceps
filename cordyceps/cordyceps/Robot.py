import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import math
import threading
import time


class Robot:
    def __init__(self, x, y, theta, name, node: Node) -> None:
        self.node = node
        self.name = name
        self.lock = threading.Lock()

        self.pose_sub = self.node.create_subscription(
            Odometry, f"/{self.name}/odom", self.odom_callback, 10
        )
        self.cmd_vel_pub = self.node.create_publisher(
            Twist, f"/{self.name}/cmd_vel", 10
        )

        self.pose = np.array([[float(x), float(y), float(theta)]]).T
        self.LOOKAHEAD = 3  # number of points pure pursuit looks ahead

        self._prev_point_index = 0

    def set_prev_point_index(self, prev_point_index: int):
        self._prev_point_index = prev_point_index

    def get_prev_point_index(self):
        return self._prev_point_index

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.pose[0][0] = round(msg.pose.pose.position.x, 2)
            self.pose[1][0] = round(msg.pose.pose.position.y, 2)

            w, x, y, z = (
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            )
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = round(math.atan2(siny_cosp, cosy_cosp), 2)
            self.pose[2][0] = yaw

    def project_pose(self, prev_point_index, route: list):
        """returns the index of the closest point in the route to the coordinates"""

        # calculate displacements between the prev_point and the nearby next
        coordinates = np.array((self.pose[0][0], self.pose[1][0]))
        route_slice = route[prev_point_index:prev_point_index + self.LOOKAHEAD + 5]
        displacements = np.linalg.norm(coordinates - np.array(route_slice), axis=1)

        # calculate the index of the closest
        min_displacement_index = prev_point_index + np.argmin(displacements)

        return min_displacement_index

    def calculate_carrot(self, projected_point_index: int, route: list):
        """given the projected position of a bot in its route, returns the goal (aka lookahead point)"""

        carrot = route[projected_point_index + self.LOOKAHEAD]
        return carrot

    def get_point_ref_to_robot_frame(self, point: np.array([[float, float, float]]).T):
        with self.lock:
            rev_point = -self.pose

            trans_matrix = np.array(
                [[1, 0, rev_point[0][0]], [0, 1, rev_point[1][0]], [0, 0, 1]]
            )
            new_point = np.matmul(trans_matrix, point)
            rot_matrix = np.array(
                [
                    [np.cos(rev_point[2][0]), -np.sin(rev_point[2][0]), 0],
                    [np.sin(rev_point[2][0]), np.cos(rev_point[2][0]), 0],
                    [0, 0, 1],
                ]
            )
            new_point = np.matmul(rot_matrix, new_point)
            return new_point

    def get_deltas(self, goal: np.array([[float, float, float]]).T) -> float:

        goal = self.get_point_ref_to_robot_frame(goal)
        
        displacement = np.hypot(goal[0], goal[1], dtype=float)

        if displacement == 0.0:
            return 0.0, 0.0, 0.0
        if goal[1] == 0:
            return displacement, 0.0, displacement

        radius = displacement**2 / (2 * goal[1])
        delta_theta = 2 * np.arcsin(displacement / (2 * radius)) * np.sign(goal[0]) # sign value indicates if the point is infront or behind
        delta_s = delta_theta * radius 
        return delta_s, delta_theta, displacement

    def publish_velocity(self, lin_vel: float, ang_vel: float):
        msg = Twist()
        msg.linear.x = lin_vel
        msg.angular.z = ang_vel

        self.cmd_vel_pub.publish(msg)

    def get_pose(self):
        with self.lock:
            return self.pose
