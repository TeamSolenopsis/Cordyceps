import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import threading
from queue import Queue
from geometry_msgs.msg import Pose

from rclpy.action import ActionClient

from cordyceps_interfaces.action import Controller
from cordyceps_interfaces.srv import CustomPathPlanner, CustomRobotAssembler
from cordyceps_interfaces.msg import RobotPaths, Task

class Vs_manager(Node):

    def __init__(self):
        super().__init__('vs_manager')
        self.task_queue = Queue(1)
        self.task_thread = threading.Thread(target=self.task_executor)
        self.task_thread.start()

        self.robot_path_client = self.create_client(CustomPathPlanner, 'get_robot_paths')
        self.assembler_client = self.create_client(CustomRobotAssembler, 'get_robot_vs_ref_pose')
        self.controller_action_client = ActionClient(self, Controller, 'controller')
        self.task_subscriber = self.create_subscription(Task, 'vs_manager/task', self.task_callback, 10)

        self.assembler_response = self.send_assembler_request(self.construct_mock_task())
        self.path_planner_response = self.send_path_planner_request(self.construct_mock_task(), self.assembler_response.vs_ref_pose)
        self.assembler_response = self.send_assembler_request(self.construct_mock_task())
        self.plot_path(self.path_planner_response, True)



    def task_callback(self, msg:Task):
        self.task_queue.put(msg)

    def send_path_planner_request(self, task, vs_ref_pose):
        planner_request = CustomPathPlanner.Request()

    def task_executor(self):
        while True:
            task = self.task_queue.get(block=True)
            print("Task received")
            paths = self.request_paths(task)
            self.controll_vs(paths)


    def request_paths(self, task:Task):
        req  = CustomPathPlanner.Request()
        req.task = task
        req.vs_ref_pose = vs_ref_pose

        future = self.robot_path_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_assembler_request(self, task):
        assembler_request = CustomRobotAssembler.Request()

        assembler_request.task = task

        response = self.assembler_client.call_async(assembler_request)

        rclpy.spin_until_future_complete(self, response)
        return response.result()

    def plot_path(self, path, show:bool) -> None:

        path_r1 = []
        path_r2 = []
        path_r3 = []
        path_r4 = []

        for poses in self.path_planner_response.robot_paths.paths[0].robot_poses[:]:
            path_r1.append([poses.x, poses.y])

        for poses in self.path_planner_response.robot_paths.paths[1].robot_poses[:]:
            path_r2.append([poses.x, poses.y])
        
        for poses in self.path_planner_response.robot_paths.paths[2].robot_poses[:]:
            path_r3.append([poses.x, poses.y])

        for poses in self.path_planner_response.robot_paths.paths[3].robot_poses[:]:
            path_r4.append([poses.x, poses.y])
            
        plt.scatter(*zip(*path_r1), s=3)
        plt.scatter(*zip(*path_r2), s=3)
        plt.scatter(*zip(*path_r3), s=3)
        plt.scatter(*zip(*path_r4), s=3)

        plt.legend(['R1', 'R2', 'R3', 'R4'])

        if show:       
            plt.show()

    def construct_mock_task(self) -> Task:
        task = Task()
        start_pose = Pose()
        goal_pose = Pose()
        task.start_pose = start_pose
        task.goal_pose = goal_pose
        task.number_of_robots = 4
        task.diameter = 2
        return task

    def controll_vs(self, paths: RobotPaths):
        goal_msg = Controller.Goal()
        goal_msg.robot_paths = paths

        self.controller_action_client.wait_for_server()
        future = self.controller_action_client.send_goal_async(goal_msg)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    vs_manager = Vs_manager()

    rclpy.spin(vs_manager)
    vs_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()