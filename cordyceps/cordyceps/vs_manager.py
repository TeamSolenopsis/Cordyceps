import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import threading
from queue import Queue
from geometry_msgs.msg import Pose

from rclpy.action import ActionClient

from cordyceps_interfaces.action import Controller
from cordyceps_interfaces.srv import CustomPathPlanner
from cordyceps_interfaces.msg import RobotPaths, Task

class Vs_manager(Node):

    def __init__(self):
        super().__init__('vs_manager')
        self.task_queue = Queue(1)
        self.task_thread = threading.Thread(target=self.task_executor)
        self.task_thread.start()

        self.robot_path_client = self.create_client(CustomPathPlanner, 'get_robot_paths')
        self.controller_action_client = ActionClient(self, Controller, 'controller')
        self.task_subscriber = self.create_subscription(Task, 'vs_manager/task', self.task_callback, 10)

    def task_callback(self, msg:Task):
        self.task_queue.put(msg)


    def task_executor(self):
        while True:
            task = self.task_queue.get(block=True)
            print("Task received")
            paths = self.request_paths(task)
            self.controll_vs(paths)


    def request_paths(self, task:Task):
        req  = CustomPathPlanner.Request()
        req.task = task

        future = self.robot_path_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def plot_path(self, path, show:bool) -> None:
        li_r1 = []
        li_r2 = []
        li_r3 = []
        li_r4 = []
        for poses in path:
            li_r1.append([poses[0][0], poses[0][1]])
            li_r2.append([poses[1][0], poses[1][1]])
            li_r3.append([poses[2][0], poses[2][1]])
            li_r4.append([poses[3][0], poses[3][1]])
            
        plt.scatter(*zip(*li_r1), s=3)
        plt.scatter(*zip(*li_r2), s=3)
        plt.scatter(*zip(*li_r3), s=3)
        plt.scatter(*zip(*li_r4), s=3)

        plt.legend(['R1', 'R2', 'R3', 'R4'])

        if show:       
            plt.show()

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