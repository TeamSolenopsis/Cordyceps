import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
import threading
from .Robot import Robot
from std_srvs.srv import Trigger
from cordyceps_interfaces.msg import RobotPaths
from cordyceps_interfaces.srv import Controller, CheckThread


class ControllerService(Node):
    def __init__(self, fleet_size=4):
        """Constructor for the ControllerService class. Initializes the ROS2 node and creates the service."""

        super().__init__("cordyceps_controller")
        self.start_follow_path_service = self.create_service(
            Controller, "start_follow_path", self.start_thread_callback
        )
        self.check_thread_state = self.create_service(
            CheckThread, "check_thread_state", self.check_thread_state_callback
        )

        # Constants delcaration
        self.MAX_BOT_SPEED = 0.2  # m/s
        self.GOAL_RADIUS = 0.0  # m
        self.robots = []
        for i in range(fleet_size):
            self.robots.append(Robot(0, 0, 0, f"r{i}", self))
        self.follow_paths_thread = None

    def start_thread_callback(self, request, response):
        """Recieves each bot's path and sends the velocities to follow them
        
        :param Request request: Request message for the service.
        :param Response response: Response message for the service."""

        self.get_logger().info("Executing goal...")

        routes = []
        for i, route in enumerate(request.robot_paths.paths):
            routes.append([])
            for poses in route.robot_poses[:]:
                routes[i].append([poses.x, poses.y])

        #self.plot_path(routes)
        self.follow_paths_thread = threading.Thread(
            target=self.follow_paths, args=(routes,)
        )
        self.follow_paths_thread.start()
        return response

    def check_thread_state_callback(self, request, response):
        """Checks if the thread is still running
        
        :param Request request: Request message for the service.
        :param Response response: Response message for the service.
        
        :returns: True if the thread is still running, False otherwise."""

        self.follow_paths_thread.join(0.06)
        response.is_alive = self.follow_paths_thread.is_alive()
        return response

    def plot_path(self, robot_paths: list[list[tuple[float, float]]]) -> None:
        """Plots the paths of the robots

        :param list[list[tuple[float, float]]] robot_paths: List of paths for each robot."""

        labels = []

        for i, path in enumerate(robot_paths):
            labels.append(f"R{i+1}")

        for path in robot_paths:
            plt.scatter(*zip(*path), s=3)

        plt.legend(labels)
        plt.show()

    def follow_paths(self, paths: list[list[tuple[float, float]]]):
        """Publishes the velocities so that the robots follow their paths

        :param list[list[tuple[float, float]]] paths: List of paths for each robot."""

        paths = np.array(paths)

        goals_achieved = False
        while not goals_achieved:
            max_distance = 0
            distances = []
            thetas = []
            first = True
            min_current_point_index = min(
                robot.project_pose(path)
                for robot, path in zip(self.robots, paths)
            )
            for robot, path in zip(
                self.robots, paths
            ):  # get deltas of each bot from their carrots
                current_point_index = robot.project_pose(path)

                goal = robot.calculate_carrot(min_current_point_index, path)
                goal = np.array((goal[0], goal[1], 1)).T  # formatting for get_deltas()
                delta_s, theta, displacement = robot.get_deltas(goal)
                if first:  # DEBUG
                    # print(f"DeltaS: {delta_s}, Displacement_index: {current_point_index}")
                    first = False

                # if displacement > self.GOAL_RADIUS:
                if delta_s > max_distance:
                    max_distance = delta_s
                distances.append(delta_s)
                thetas.append(theta)
                # else:
                #     distances.append(0)
                #     thetas.append(theta)

                robot.set_prev_point_index(current_point_index)

            bot_velocities = self.calc_velocities(distances, thetas, max_distance)
            if all(velocity == [0, 0] for velocity in bot_velocities):
                goals_achieved = True

            for robot, velocity in zip(
                self.robots, bot_velocities
            ):  # publish velocity commands to each bot
                print(bot_velocities)
                robot.publish_velocity(float(velocity[0]), float(velocity[1]))

    def calc_velocities(self, distances:list[float], thetas:list[float], max_distance:float) -> list[list[float]]:
        """Calculates the velocities for each robot

        :param list[float] distances: List of distances from the robots to their goals.
        :param list[float] thetas: List of angles from the robots to their goals.
        :param float max_distance: largest distance a robot travels.

        :returns: List of velocities for each robot."""

        bot_velocities = []

        for delta_s, delta_theta in zip(distances, thetas):
            if delta_s == 0:
                bot_velocities.append([0, 0])
            else:
                delta_t = max_distance / self.MAX_BOT_SPEED
                self.time = delta_t
                lin_vel = delta_s / delta_t
                ang_vel = delta_theta / (delta_t)
                bot_velocities.append([lin_vel, ang_vel])

        return bot_velocities


def main(args=None):
    rclpy.init(args=args)
    controller_action_server = ControllerService()
    rclpy.spin(controller_action_server)
    controller_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
