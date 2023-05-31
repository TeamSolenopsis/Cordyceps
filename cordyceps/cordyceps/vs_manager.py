import rclpy
from rclpy.node import Node
import threading
from queue import Queue
from geometry_msgs.msg import Pose

from rclpy.action import ActionClient

from cordyceps_interfaces.action import Controller
from cordyceps_interfaces.srv import CustomPathPlanner, CustomRobotAssembler
from cordyceps_interfaces.msg import RobotPaths, Task, RobotPose,Path

class Vs_manager(Node):

    def __init__(self):
        super().__init__('vs_manager')
        self.task_queue = Queue(1)
        self.task_thread = threading.Thread(target=self.task_executor)
        self.task_thread.start()

        self.robot_path_client = self.create_client(CustomPathPlanner, 'get_robot_paths')
        self.assembler_client = self.create_client(CustomRobotAssembler, 'get_robot_vs_ref_pose')
        self.controller_action_client = ActionClient(self, Controller, 'controller')

        # msg = RobotPaths()
        # path = Path()
        # pose = RobotPose()
        # pose.x = 2.0
        # pose.y = 0.0
        # path.robot_poses = []
        # path.robot_poses.append(pose)
        # msg.paths = []
        # msg.paths.append(path)
        # self.controll_vs(msg)

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
        assembler_request = CustomRobotAssembler.Request()
        assembler_request.task = task
        while True:
            if self.assembler_client.wait_for_service():
                break
        response = self.assembler_client.call(assembler_request)
        return response.vs_ref_pose

    def construct_mock_task(self) -> Task:
        task = Task()
        start_pose = Pose()
        start_pose.position.x = 1.0
        start_pose.position.y = 0.0
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
        response = self.controller_action_client.send_goal(goal_msg)
        print(response)
        return response


def main(args=None):
    rclpy.init(args=args)
    vs_manager = Vs_manager()

    rclpy.spin(vs_manager)
    vs_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()