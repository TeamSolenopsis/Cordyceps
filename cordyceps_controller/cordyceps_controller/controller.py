import rclpy
from rclpy.node import Node

class Controller(Node):
    def __init__(self):
        super().__init__('cordyceps_controller')
        

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

