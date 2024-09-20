import rclpy
from rclpy.node import Node
import signal

class ContextManagerNode(Node):
    def __init__(self):
        super().__init__('context_manager_node')


def main(args=None):
    rclpy.init(args=args)
    node = ContextManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()