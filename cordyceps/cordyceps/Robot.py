import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import math
import threading
import time
import json
import paho.mqtt.client as mqtt

class Robot:
    def __init__(self, x, y, theta, name) -> None:
        self.name = name
        self.lock = threading.Lock()

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_connect = self.on_connect

        # TODO: add this to a config file
        self.mqtt_client.connect("192.168.0.101", 1883, 60)
        self.mqtt_client.loop_start()

        self.pose = np.array([[float(x),float(y),float(theta)]]).T

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(f'/{self.name}/odom')

    def on_message(self, client, userdata, msg):
        with self.lock:
            json_odom_msg = json.loads(msg.payload)

            self.pose[0][0] = json_odom_msg['position']['x']
            self.pose[1][0] = json_odom_msg['position']['y']
            self.pose[2][0] = json_odom_msg['orientation']['w']


    def get_point_ref_to_robot_frame(self, point:np.array([[float, float, float]]).T):
        with self.lock:
            rev_point = -self.pose

            trans_matrix = np.array([[1, 0, rev_point[0][0]], [0, 1, rev_point[1][0]], [0, 0, 1]])
            new_point = np.matmul(trans_matrix, point)
            rot_matrix = np.array([[np.cos(rev_point[2][0]), -np.sin(rev_point[2][0]), 0], [np.sin(rev_point[2][0]), np.cos(rev_point[2][0]), 0], [0, 0, 1]])
            new_point = np.matmul(rot_matrix, new_point)
            return new_point

    def get_deltas(self, goal:np.array([[float,float,float]]).T) -> float:
        goal = self.get_point_ref_to_robot_frame(goal)
        displacement = float(np.sqrt(goal[0][0]**2 + goal[1][0]**2))

        if displacement == 0.0:
            return 0.0, 0.0, 0.0
        if goal[1][0] == 0:
            return displacement,0.0, displacement

        radius = displacement **2 / (2 * goal[1][0])
        delta_theta = 2 * np.arcsin(displacement/ (2 * radius))
        delta_s = delta_theta * radius
        return delta_s, delta_theta, displacement
    
    def publish_velocity(self, lin_vel:float, ang_vel:float):
        cmd_vel = {
            'linear': lin_vel,
            'angular': ang_vel
        }

        self.mqtt_client.publish(f'/{self.name}/cmd_vel', json.dumps(cmd_vel))

    def get_pose(self):
        with self.lock:
            return self.pose
