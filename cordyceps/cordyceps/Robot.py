from rclpy.node import Node

class Robot():
    def __init__(self, name, offset_x, offset_y, node:Node) -> None:
        self.name = name
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.node = node
        self.cmd_vel_publisher = None




