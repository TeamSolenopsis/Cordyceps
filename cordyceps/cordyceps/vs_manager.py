import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import time
import string

from cordyceps_interfaces.msg import arrived
from cordyceps_interfaces.msg import pose_shape
from cordyceps_interfaces.msg import assembled  

class Vs_manager(Node):

    def __init__(self):
        super().__init__('vs_manager')
        
        self.publisher_ = self.create_publisher(pose_shape, 'topic', 10) 
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.subscription = self.create_subscription(
            arrived,                                               
            'topic',
            self.listener_callback,
            10)
        self.subscription

        self.subscription = self.create_subscription(
            assembled,                                               
            'topic',
            self.assembled_callback,
            10)
        self.subscription

    def listener_callback(self, msg:arrived):
            self.get_logger().info('I heard: "%d"' % msg.arrived) 

    def assembled_callback(self, msg:assembled):
            self.get_logger().info('I heard: "%d"' % msg.assembled) 

    def timer_callback(self):
        msg = pose_shape()                                                
        msg.pose_shape.number_of_robots = 4
        msg.pose_shape.pose.x = 2
        msg.pose_shape.pose.y = 2
        msg.pose_shape.pose.angular.z = 0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.pose)       
        self.i += 1



def main(args=None):
    rclpy.init(args=args)
    vs_manager = Vs_manager()
    rclpy.spin(vs_manager)
    vs_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()