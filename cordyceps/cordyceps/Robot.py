from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import Odometry
from cordyceps_interfaces.msg import RobotPose
from geometry_msgs.msg import Pose2D
import numpy as np
from math import cos, sin



class Robot:
    def __init__(self, name, node: Node) -> None:
        self.node = node
        self.name = name
        self.cmd_vel_publisher = self.node.create_publisher(
            Twist, f"/{self.name}/cmd_vel", 10
        )
        self.odom_subscriber = self.node.create_subscription(
            Odometry, f"/{self.name}/odom", self.pose_callback, 10
        )
        self.pose = Pose2D()

    def pose_callback(self, msg: Odometry) -> None:
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta = np.arctan2(2*(msg.pose.pose.orientation.w*msg.pose.pose.orientation.z + msg.pose.pose.orientation.x*msg.pose.pose.orientation.y), 1 - 2*(msg.pose.pose.orientation.y**2 + msg.pose.pose.orientation.z**2))

    def get_pose(self) -> RobotPose:
        return self.pose

    def publish_velocity(self, vel: Twist) -> None:
        self.cmd_vel_publisher.publish(vel)

    def compute_deltas(self, goal_point):
        x, y = goal_point
        goal_distance = np.sqrt(x**2 + y**2)
        radius = goal_distance / (-2 * y) if y != 0 else 0
        delta_theta = 2 * np.arcsin(goal_distance / (2 * radius)) if radius != 0 else np.inf
        delta_s = delta_theta * radius  # orthodromic distance
        return delta_s, delta_theta

    def transform_frame(self, point:RobotPose, new_frame):
        """returns the coordinates of the provided point in reference to the desired frame"""

        frame_xy = np.array([new_frame.x, new_frame.y])
        theta = new_frame.theta

        point_xy = np.array([point.x, point.y])

        translated_point = point_xy - frame_xy
        rot_matrix = np.array(([cos(-theta), -sin(-theta)], [sin(-theta), cos(-theta)]))

        transformed_point = rot_matrix.dot(translated_point)

        return transformed_point  # TODO double check
