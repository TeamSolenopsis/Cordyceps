import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .Robot import Robot
from cordyceps_interfaces.msg import RobotPaths

from cordyceps_interfaces.action import Controller


class ControllerActionServer(Node):
    def __init__(self, fleet_size=1):
        super().__init__("cordyceps_controller")
        self.action_server = ActionServer(
            self, Controller, "controller", self.execute_callback
        )

        # Constants delcaration
        self.MAX_BOT_SPEED =  0.1 #m/s

        self.bots = []
        for i in range(fleet_size):
            self.bots.append(Robot(f"r{i}", self))

    def execute_callback(self, goal_handle):
        """Recieves each bot's path and sends the velocities to follow them"""

        self.get_logger().info("Executing goal...")
        self.plot_path(goal_handle.request.robot_paths)
        self.follow_path(goal_handle.request.robot_paths)
        goal_handle.succeed()
        result = Controller.Result()
        result.done = True
        return result

    def plot_path(self, robot_paths) -> None:
        path_r = []
        labels = []

        for i, path in enumerate(robot_paths.paths):
            path_r.append([])
            for poses in path.robot_poses[:]:
                path_r[i].append([poses.x, poses.y])
            labels.append(f"R{i+1}")

        for path in path_r:
            plt.scatter(*zip(*path), s=3)

        plt.legend(labels)
        plt.show()

    def follow_path(self, paths: RobotPaths):
        paths = np.array(
            paths.paths
        ).transpose()  # paths is now a list of lists of robot poses
        print(paths)
        for goals in paths:  # iterate through immeadiate goals
            goal_achieved = False
            while not goal_achieved:
                goals_tformed = [
                    bot.transform_frame(goal, bot.get_pose())
                    for goal, bot in zip(goals.robot_poses, self.bots)
                ]  # express the goals relative to the bot's frame

                velocities = self.compute_velocities(goals_tformed)
                self.__publish_velocities(velocities)
                goal_achieved = True if goals_tformed == np.array(len(goals)*np.array([0,0])) else False  # TODO give tolerances

    def compute_velocities(self, goal_points):
        """returns the required velocities for each robot to reach its goal (given the current poses) at the same time"""
        bot_path_distances = []
        bot_path_angles = []
        for goal, bot in zip(goal_points, self.bots):
            path_distance, path_angle = bot.compute_deltas(goal)
            bot_path_distances.append(path_distance)
            bot_path_angles.append(path_angle)

        LONGEST_DISTANCE = max(abs(path_distance) for path_distance in bot_path_distances)

        bot_velocities = []
        for path_distance, path_angle in zip(bot_path_distances, bot_path_angles):
            delta_t = LONGEST_DISTANCE / self.MAX_BOT_SPEED
            lin_vel = path_distance / delta_t if delta_t != 0 else 0
            ang_vel = path_angle / delta_t if delta_t != 0 else 0
            bot_velocities.append([lin_vel, ang_vel])

        return bot_velocities

    def __publish_velocities(self, bot_velocities):
        """published to each vel topic at the same time"""
        for bot, velocity in zip(self.bots, bot_velocities):
            msg = Twist()
            msg.linear.x = float(velocity[0])
            msg.angular.z = float(velocity[1])
            bot.publish_velocity(msg)


def main(args=None):
    rclpy.init(args=args)
    controller_action_server = ControllerActionServer()
    rclpy.spin(controller_action_server)
    controller_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()