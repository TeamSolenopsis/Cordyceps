import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from queue import Queue
from geometry_msgs.msg import Pose
from .path_planner import PathPlanner
from .custom_assembler import Assembler
from .vs_controller import ControllerService

from cordyceps_interfaces.srv import CustomPathPlanner, CustomRobotAssembler, Controller, CheckThread, AssemblerGetVsRefPose, CheckGoalPoseReached
from cordyceps_interfaces.msg import RobotPaths, Task, RobotPose,Path

class VsManager(Node):

    def __init__(self):
        super().__init__('vs_manager')
        self.task_queue = Queue(1)
        self.task_thread = threading.Thread(target=self.task_executor)
        self.task_thread.start()

        self.robot_path_client = self.create_client(CustomPathPlanner, 'get_robot_paths')
        self.assembler_get_vs_ref_pose_client = self.create_client(AssemblerGetVsRefPose, 'get_robot_vs_ref_pose')
        self.assembler_client = self.create_client(CustomRobotAssembler, 'assemble_robots')
        self.assembler_check_goal_reached_client = self.create_client(CheckGoalPoseReached, 'check_goal_pose_reached')
        self.start_path_follow_client = self.create_client(Controller, 'start_follow_path')
        self.check_thread_state_client = self.create_client(CheckThread, 'check_thread_state')

        self.task_subscriber = self.create_subscription(Task, 'vs_manager/task', self.task_callback, 10)

        self.timer = self.create_timer(5.0, self.timer_callback)
        self.pub = self.create_publisher(Task, 'vs_manager/task', 10)


    def timer_callback(self):
        task = self.construct_mock_task()
        self.pub.publish(task)

    def task_callback(self, msg:Task):
        self.task_queue.put(msg)

    def task_executor(self):
        while True:
            task = self.task_queue.get(block=True)
            vs_ref_pose = self.request_vs_ref_pose(task)
            paths = self.request_paths(task, vs_ref_pose)
            self.assemble_robots(task)
            self.controll_vs(paths)

    def request_paths(self, task:Task, vs_ref_pose:list):
        req  = CustomPathPlanner.Request()
        req.task = task
        req.vs_ref_pose = vs_ref_pose

        while True:
            if self.robot_path_client.wait_for_service():
                break
        respone = self.robot_path_client.call(req)
        return respone.robot_paths
    
    def request_vs_ref_pose(self, task):
        assembler_request = AssemblerGetVsRefPose.Request()
        assembler_request.task = task
        while True:
            if self.assembler_get_vs_ref_pose_client.wait_for_service():
                break
        response = self.assembler_get_vs_ref_pose_client.call(assembler_request)
        return response.vs_ref_pose
    
    def assemble_robots(self, task):
        assembler_request = CustomRobotAssembler.Request()
        assembler_request.task = task
        while True:
            if self.assembler_client.wait_for_service():
                break
        self.assembler_client.call(assembler_request)

        arrived = False
        while arrived == False:
            arrived = self.check_goal_pose_reached()

        return True
    
    def check_goal_pose_reached(self):
        assembler_request = CheckGoalPoseReached.Request()
        while True:
            if self.assembler_check_goal_reached_client.wait_for_service():
                break
        response = self.assembler_check_goal_reached_client.call(assembler_request)
        return response.all_arrived

    def construct_mock_task(self) -> Task:
        task = Task()
        start_pose = Pose()
        start_pose.position.x = 1.0
        start_pose.position.y = 1.0
        goal_pose = Pose()
        task.start_pose = start_pose
        task.goal_pose = goal_pose
        task.number_of_robots = 4
        task.diameter = 2
        return task

    def controll_vs(self, paths: RobotPaths):
        request = Controller.Request()
        request.robot_paths = paths

        self.start_path_follow_client.wait_for_service()
        self.start_path_follow_client.call(request)
        
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