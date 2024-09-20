import rclpy
from rclpy.node import Node

class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')

def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()