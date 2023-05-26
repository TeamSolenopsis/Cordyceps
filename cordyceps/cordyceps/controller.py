import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import numpy as np
import matplotlib.pyplot as plt

from cordyceps_interfaces.action import Controller

class ControllerActionServer(Node):
    def __init__(self):
        super().__init__('cordyceps_controller')
        self.action_server = ActionServer(self, Controller, 'controller', self.execute_callback)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.plot_path(goal_handle.request.robot_paths, False)
        goal_handle.succeed()
        result = Controller.Result()
        result.done = True
        return result
    
    def plot_path(self, robot_paths, show:bool) -> None:

        path_r1 = []
        path_r2 = []
        path_r3 = []
        path_r4 = []

        for poses in robot_paths.paths[0].robot_poses[:]:
            path_r1.append([poses.x, poses.y])

        for poses in robot_paths.paths[1].robot_poses[:]:
            path_r2.append([poses.x, poses.y])
        
        for poses in robot_paths.paths[2].robot_poses[:]:
            path_r3.append([poses.x, poses.y])

        for poses in robot_paths.paths[3].robot_poses[:]:
            path_r4.append([poses.x, poses.y])
            
        plt.scatter(*zip(*path_r1), s=3)
        plt.scatter(*zip(*path_r2), s=3)
        plt.scatter(*zip(*path_r3), s=3)
        plt.scatter(*zip(*path_r4), s=3)

        plt.legend(['R1', 'R2', 'R3', 'R4'])

        if show:       
            plt.show()

def main(args=None):
    rclpy.init(args=args)
    controller_action_server = ControllerActionServer()
    rclpy.spin(controller_action_server)
    controller_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()