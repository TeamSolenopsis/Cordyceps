import rclpy
from rclpy.node import Node
import numpy as np
import threading
from .Robot import Robot
from cordyceps_interfaces.srv import Controller, CheckThread


class ControllerService(Node):

    def __init__(self):
        """Constructor for the ControllerService class. Initializes the ROS2 node and creates the service."""

        super().__init__("cordyceps_controller")
        self.start_follow_route_service = self.create_service(
            Controller, "start_follow_route", self.start_thread_callback
        )
        self.check_thread_state = self.create_service(
            CheckThread, "check_thread_state", self.check_thread_state_callback
        )

        self.MAX_BOT_SPEED = 0.1  # m/s
        self.GOAL_RADIUS = 0.0  # m
        self.robots = []

        self.follow_routes_thread = None

    def start_thread_callback(self, request, response):
        """Recieves each bot's path and sends the velocities to follow them
        
        :param Request request: Request message for the service.
        :param Response response: Response message for the service."""

        self.get_logger().info(f"""
        ============================================
        | ~ Cordyceps controller executing route ~ |
        ============================================

        ~~ Parameters:  
        ~ Robot names: {[robot.name for robot in self.robots]}
        ~ Fleet size: {len(request.robot_routes.routes)}
        ~ Number of poses in route: {len(request.robot_routes.routes[0].robot_poses)}
        """)

        routes = []
        for i, route in enumerate(request.robot_routes.routes):
            self.robots.append(Robot(0, 0, 0, f"r{i}", self))
            routes.append([])
            for poses in route.robot_poses[:]:
                routes[i].append([poses.x, poses.y])

        self.follow_routes_thread = threading.Thread(
            target=self.follow_routes, args=(routes,)
        )
        self.follow_routes_thread.start()
        return response

    def check_thread_state_callback(self, request, response):
        """Checks if the thread is still running
        
        :param Request request: Request message for the service.
        :param Response response: Response message for the service.
        
        :returns: True if the thread is still running, False otherwise."""

        self.follow_routes_thread.join(0.06)
        response.is_alive = self.follow_routes_thread.is_alive()
        return response

    def follow_routes(self, routes: list[list[tuple[float, float]]]):
        """Publishes the velocities so that the robots follow their routes
        
        :param list[list[tuple[float, float]]] routes: List of routes for each robot."""
        
        routes = np.array(routes)
        route_completed = False
        while not route_completed:
            max_distance = 0
            distances = []
            thetas = []
            min_current_point_index = min(
                robot.project_pose(route)
                for robot, route in zip(self.robots, routes)
            )
            for robot, route in zip(self.robots, routes):
                current_point_index = robot.project_pose(route)

                goal = robot.calculate_carrot(min_current_point_index, route)
                goal = np.array((goal[0], goal[1], 1)).T 
                delta_s, delta_s_wheelbase, theta, displacement = robot.get_deltas(goal)


                if abs(delta_s_wheelbase) > max_distance:
                    max_distance = delta_s_wheelbase
                distances.append(delta_s)
                thetas.append(theta)

                robot.set_prev_point_index(current_point_index)

            bot_velocities = self.calc_velocities(distances, thetas, max_distance)  


            for robot, velocity in zip(self.robots, bot_velocities):  
                robot.publish_velocity(float(velocity[0]), float(velocity[1]))
            if current_point_index == len(route) - 1:
                for robot in self.robots:
                    robot.publish_velocity(0.0, 0.0)

                route_completed = True   

    def calc_velocities(self, distances:list[float], thetas:list[float], max_distance:float) -> list[list[float]]:
        """Calculates the velocities for each robot

        :param list[float] distances: List of distances from the robots to their goals.
        :param list[float] thetas: List of angles from the robots to their goals.
        :param float max_distance: largest distance a robot travels.

        :returns: List of velocities for each robot."""

        bot_velocities = []

        delta_s = np.array(distances)
        delta_theta = np.array(thetas)
        delta_t = abs(max_distance) / self.MAX_BOT_SPEED
        self.time = delta_t

        non_zero_indices = np.where(delta_s != 0)

        lin_vel = np.zeros_like(delta_s)
        lin_vel[non_zero_indices] = delta_s[non_zero_indices] / delta_t

        ang_vel = np.zeros_like(delta_theta)
        ang_vel[non_zero_indices] = delta_theta[non_zero_indices] / delta_t

        bot_velocities = np.column_stack((lin_vel, ang_vel)).tolist()

        return bot_velocities

def main(args=None):
    rclpy.init(args=args)
    controller_action_server = ControllerService()
    rclpy.spin(controller_action_server)
    controller_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
