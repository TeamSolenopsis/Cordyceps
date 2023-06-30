import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from queue import Queue
from geometry_msgs.msg import Pose
from .path_planner import PathPlanner
from .vs_assembler import Assembler
from .vs_controller import ControllerService
from cordyceps_interfaces.srv import CustomPathPlanner, CustomRobotAssembler, Controller, CheckThread
from cordyceps_interfaces.msg import RobotRoutes, Task

class VsManager(Node):

    def __init__(self):
        """Constructor for the VsManager class. Initializes the ROS2 node and creates the service."""

        super().__init__('vs_manager')
        self.task_queue = Queue(1)
        self.task_thread = threading.Thread(target=self.task_executor)
        self.task_thread.start()

        self.robot_route_client = self.create_client(CustomPathPlanner, 'get_robot_routes')
        self.assembler_client = self.create_client(CustomRobotAssembler, 'get_robot_vs_ref_pose')
        self.start_route_follow_client = self.create_client(Controller, 'start_follow_route')
        self.check_thread_state_client = self.create_client(CheckThread, 'check_thread_state')

        self.task_subscriber = self.create_subscription(Task, 'vs_manager/task', self.task_callback, 10)

        self.timer = self.create_timer(5.0, self.timer_callback)
        self.pub = self.create_publisher(Task, 'vs_manager/task', 10)


    def timer_callback(self):
        """Callback function for the timer. Publishes a mock task to the task topic."""

        task = self.construct_mock_task()
        self.pub.publish(task)

    def task_callback(self, msg:Task):
        """Callback function for the task subscriber. Adds the task to the task queue.
        
        :param Task msg: The task message that is received from the task topic."""

        self.task_queue.put(msg)

    def task_executor(self):
        """Function that executes the tasks in the task queue."""

        while True:
            task = self.task_queue.get(block=True)
            vs_ref_pose = self.request_vs_ref_pose(task)
            routes = self.request_routes(task, vs_ref_pose)
            self.controll_vs(routes)            

    def request_routes(self, task:Task, vs_ref_pose:list):
        """Function that requests the routes from the path planner service.
        
        :param Task task: The task for which the routes are requested.
        :param list vs_ref_pose: The reference poses for each robot.
        
        :returns: The routes for each robot."""

        req = CustomPathPlanner.Request()
        req.task = task
        req.vs_ref_pose = vs_ref_pose

        while True:
            if self.robot_route_client.wait_for_service():
                break
        
        respone = self.robot_route_client.call(req)
        return respone.robot_routes
    
    def request_vs_ref_pose(self, task):
        """Function that requests the reference poses for each robot from the assembler service.
        
        :param Task task: The task for which the reference poses are requested.
        
        :returns: The reference poses for each robot."""

        assembler_request = CustomRobotAssembler.Request()
        assembler_request.task = task
        while True:
            if self.assembler_client.wait_for_service():
                break
        response = self.assembler_client.call(assembler_request)
        return response.vs_ref_pose

    def construct_mock_task(self) -> Task:
        """Function that constructs a mock task.
        
        :returns: A mock task."""

        task = Task()
        start_pose = Pose()
        start_pose.position.x = 2.0
        start_pose.position.y = 2.0
        goal_pose = Pose()
        task.start_pose = start_pose
        task.goal_pose = goal_pose
        task.number_of_robots = 4
        task.diameter = 2

        return task

    def controll_vs(self, routes: RobotRoutes):
        """Function that calls the controller service to start the path following.
        
        :param RobotPaths paths: The paths for each robot.
        """

        request = Controller.Request()
        request.robot_routes = routes

        self.start_route_follow_client.wait_for_service()
        
        self.start_route_follow_client.call(request)
        
        is_alive = True
        request = CheckThread.Request()

        while is_alive:
            is_alive = self.check_thread_state_client.call(request).is_alive


def main(args=None):
    rclpy.init(args=args)
    vs_manager = VsManager()
    controller = ControllerService()
    planner = PathPlanner()
    assembler = Assembler()

    executor = MultiThreadedExecutor()
    executor.add_node(vs_manager)
    executor.add_node(controller)
    executor.add_node(planner)
    executor.add_node(assembler)

    executor.spin()
    vs_manager.destroy_node()
    controller.destroy_node()
    planner.destroy_node()
    assembler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()