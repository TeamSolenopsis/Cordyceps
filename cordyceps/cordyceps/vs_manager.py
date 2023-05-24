import rclpy
from rclpy.node import Node

# from cordyceps_interfaces.msg import arrived
# from cordyceps_interfaces.msg import pose_shape
# from cordyceps_interfaces.msg import assembled  
class Vs_manager(Node):

    def __init__(self):
        super().__init__('vs_manager')
       

def main(args=None):
    rclpy.init(args=args)
    vs_manager = Vs_manager()
    rclpy.spin(vs_manager)
    vs_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()