import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

from rclpy.action import ActionClient


from cordyceps_interfaces.action import Controller

from geometry_msgs.msg import Pose

from cordyceps_interfaces.srv import CustomPathPlanner, CustomRobotAssembler
from cordyceps_interfaces.msg import Path, RobotPaths, RobotPose, Task

class Vs_manager(Node):

    def __init__(self):
        super().__init__('vs_manager')

        self.path_planner_client = self.create_client(CustomPathPlanner, 'get_robot_paths')
        self.assembler_client = self.create_client(CustomRobotAssembler, 'get_robot_vs_ref_pose')
        
        self.assembler_response = self.send_assembler_request(self.construct_mock_task())
        self.path_planner_response = self.send_path_planner_request(self.construct_mock_task(), self.assembler_response.vs_ref_pose)
        
        self.plot_path(self.path_planner_response, True)

    def send_path_planner_request(self, task, vs_ref_pose):
        planner_request = CustomPathPlanner.Request()

        planner_request.task = task
        planner_request.vs_ref_pose = vs_ref_pose

        response = self.path_planner_client.call_async(planner_request)

        rclpy.spin_until_future_complete(self, response)
        return response.result()
    
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
        return self.controller_action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    vs_manager = Vs_manager()

    rclpy.spin(vs_manager)
    vs_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()