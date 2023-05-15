import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import time

class Controller(Node):
    def __init__(self):
        super().__init__('cordyceps_controller')
        
        # Inputs: cmd_vel, path, odom for each robot
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Pose, '/path', self.path_callback, 10)

        # TODO: Change subscription to service call
        self.create_subscription(Odometry, 'r1/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, 'r2/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, 'r3/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, 'r4/odom', self.odom_callback, 10)
        
        # Outputs: cmd_vel for each robot
        self.pub_r1 = self.create_publisher(Twist, 'r1/cmd_vel', 10)
        self.pub_r2 = self.create_publisher(Twist, 'r2/cmd_vel', 10)
        self.pub_r3 = self.create_publisher(Twist, 'r3/cmd_vel', 10)
        self.pub_r4 = self.create_publisher(Twist, 'r4/cmd_vel', 10)

        self.m2p = 3779.52755


        self.RESOLUTION = 1000 # The amount of points in which the paths will be split.
        self.MAX_SPEED = 20  # Maximum allowed speed from a robot.

        self.x = np.linspace(1.0, -100.0, self.RESOLUTION)
        self.y_lins = np.linspace(1.0, 100.0, self.RESOLUTION)
        self.y = np.sin(self.y_lins / 2) * 5
        self.vs_origin_x = 960  # pixels
        self.vs_origin_y = 540  # pixels
        self.angle = 0.0  # rad

        self.robot_paths = self.generate_paths()

    def cmd_vel_callback(self, msg):
        #TODO: Implement dividing the cmd_vel into 4 cmd_vel for each robot 
        print(msg)
        
    def path_callback(self, msg):
        #TODO: Implement dividing the pose into 4 pose for each robot
        self.robot_paths = self.generate_paths()

    def odom_callback(self, msg):
        pass

    def generate_paths(self) -> dict:
        path = []
        for i in range(self.RESOLUTION - 1):
            # mag_vel = np.sqrt(np.square(x[i+1]-x[i]) + np.square(y[i+1]-y[i]))
            x_goal = self.x[i + 1] - self.x[i]
            y_goal = self.y[i + 1] - self.y[i]

            angle = np.arctan(y_goal / x_goal)
            
            if y_goal < 0 and x_goal < 0:
                    angle += np.pi
            if y_goal > 0 and x_goal < 0:
                    angle -= np.pi
            #if y_goal < 0:  # check if the angle is above pi.
            #    angle += np.pi

            tf_matrix = np.array(
                [
                    [np.cos(angle), -np.sin(angle), self.x[i] * self.MAX_SPEED],
                    [np.sin(angle), np.cos(angle), self.y[i] * self.MAX_SPEED],
                    [0, 0, 1],
                ]
            )  # transformation matrix template.

            # Robot coordinates.
            bot_0_xy = np.array([50, -50, 1])
            bot_1_xy = np.array([50, 50, 1])
            bot_2_xy = np.array([-50, 50, 1])
            bot_3_xy = np.array([-50, -50, 1])

            # Calculation for pose of every robot in the VS.
            trans_0 = tf_matrix.dot(bot_0_xy)
            trans_1 = tf_matrix.dot(bot_1_xy)
            trans_2 = tf_matrix.dot(bot_2_xy)
            trans_3 = tf_matrix.dot(bot_3_xy)

            # Calculated path for every robot in the VS.
            path.append(
                (
                    ([trans_0[0] + self.vs_origin_x, trans_0[1] + self.vs_origin_y, angle]),
                    ([trans_1[0] + self.vs_origin_x, trans_1[1] + self.vs_origin_y, angle]),
                    ([trans_2[0] + self.vs_origin_x, trans_2[1] + self.vs_origin_y, angle]),
                    ([trans_3[0] + self.vs_origin_x, trans_3[1] + self.vs_origin_y, angle]),
                )
            )
    

        for vs_pose_id, vs_pose in enumerate(path):
            for bot_pose_id, bot_pose in enumerate(vs_pose):
                try:
                    yy = path[vs_pose_id + 1][bot_pose_id][1] - bot_pose[1]
                    xx = path[vs_pose_id + 1][bot_pose_id][0] - bot_pose[0]
                except:
                    alpha=alpha
                alpha = -np.arctan(yy / xx)

                if yy < 0 and xx < 0:
                    alpha += np.pi
                if yy > 0 and xx < 0:
                    alpha -= np.pi

                bot_pose[2] = alpha

        self.cmd_vels = self.calculate_cmd_vel_robot(path=path)

        return path        

    def calculate_cmd_vel_robot(self, path) -> Twist:
        #TODO: Calculate the cmd_vel for the robot based on the current pose and the goal pose
        angle_cof = 1
        lin_cof = 10

        for pose_index, poses in enumerate(path):

            time.sleep(0.01)

            print(f'pose {pose_index}')
            for robot_index, robot_pose in enumerate(poses):
                try:
                    d_pose_x = (path[pose_index + 1][robot_index][0]) - robot_pose[0]
                    d_pose_y = (path[pose_index + 1][robot_index][1]) - robot_pose[1]

                    #d_pose_angle = np.arctan(d_pose_y / d_pose_x) 
                    d_pose_angle = robot_pose[2]

                    net_speed = np.sqrt(np.square(d_pose_x) + np.square(d_pose_y))

                except:
                    print('EXCEPTION 140 CALLED')
                    d_pose_x=d_pose_x

                cmd_vel = Twist()  

                if(robot_index == 0):
                    cmd_vel.angular.z = d_pose_angle / angle_cof
                    cmd_vel.linear.x = net_speed / lin_cof
                    self.pub_r1.publish(cmd_vel)

                if(robot_index == 1):
                    cmd_vel.angular.z = d_pose_angle / angle_cof
                    cmd_vel.linear.x = net_speed / lin_cof
                    self.pub_r2.publish(cmd_vel)

                if(robot_index == 2):
                    cmd_vel.angular.z = d_pose_angle / angle_cof
                    cmd_vel.linear.x = net_speed / lin_cof
                    self.pub_r3.publish(cmd_vel)

                if(robot_index == 3):
                    cmd_vel.angular.z = d_pose_angle / angle_cof
                    cmd_vel.linear.x = net_speed / lin_cof
                    self.pub_r4.publish(cmd_vel)

                

                print(f'r{robot_index} d_pose: {str(d_pose_x)[:5]}, {str(d_pose_y)[:5]}, d_angle {str(d_pose_angle)[:5]}')               


                # print(f'Pose-{pose_index}, r{robot_index}: {robot_pose}')

        
        return Twist()


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()