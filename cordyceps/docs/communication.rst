Messages
--------

Path
^^^^
    * robot_poses : Robot_Pose[] --> List of poses

Virtual structure path contains list of poses

RobotPose
^^^^^^^^^
    * x : float32 --> x postion of the robot
    * y : float32 --> y postion of the robot

Pose of a robot 

RobotRoute
^^^^^^^^^^
    * routes : Path[] --> List of paths

Object which contains a list of paths for each robot.

Task
^^^^
    * start_pose : geometry_msgs/Pose --> Start pose of the robot
    * goal_pose : geometry_msgs/Pose --> Goal pose of the robot
    * number_of_robots : int64 --> Number of robots
    * diameter : float32 --> Diameter of the circle on which the robots are placed around the vs center point in meters

Message to start the controller with given parameters

Services
--------

CheckThread
^^^^^^^^^^^
    * request : NULL
    * response : bool

response with true if the thread is running, false if not.

Controller
^^^^^^^^^^
    * request : RobotRoutes
    * response : NULL

Start the controller with the given routes.

CustomPathPlanner
^^^^^^^^^^^^^^^^^
    * request : Task, RobotPose[]
    * response : RobotRoutes

Response with routes for each robot based on the given task and robot poses.

CustomRobotAssembler
^^^^^^^^^^^^^^^^^^^^
    * request : Task
    * response : RobotPose[]

Response with the vs reference poses based on the given task.