import rclpy
from rclpy.node import Node
from cordyceps_interfaces.msg import Task
from geometry_msgs.msg import Pose

class TaskPublisher(Node):
    def __init__(self):
        super().__init__('task_publisher')
        self.publisher = self.create_publisher(Task, 'vs_manager/task', 10)

        timer_period_seconds = 5.0 
        self.timer = self.create_timer(timer_period_seconds, self.timer_callback)
        
    def timer_callback(self):
        """Callback function for the timer. Publishes a mock task to the task topic. 
        Used for debugging and testing. should be triggered by an action"""

        task = self.construct_mock_task()
        self.publisher.publish(task)

    def construct_mock_task(self) -> Task:
        """Function that constructs a mock task. Used for debugging and testing.
        
        :returns: A mock task."""

        task = Task()

        start_pose = Pose()
        start_pose.position.x = 0.0
        start_pose.position.y = 0.0

        goal_pose = Pose()
        task.start_pose = start_pose
        task.goal_pose = goal_pose

        task.number_of_robots = 3
        task.diameter = 1.0             # diameter of the circle on which the robots are placed around the vs center point in meters

        return task
    
def main(args=None):
    rclpy.init(args=args)
    task_publisher = TaskPublisher()
    rclpy.spin(task_publisher)
    task_publisher.destroy_node()
    rclpy.shutdown()