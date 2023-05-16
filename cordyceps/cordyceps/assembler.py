from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

class Assambler(Node):
    def __init__(self):
        super().__init__('assambler')
        self._logger = self.get_logger()
        self._logger.info('Assambler node started')


def main(args=None):
    rclpy.init(args=args)
    assembler = Assambler()
    rclpy.spin(assembler)
    assembler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()