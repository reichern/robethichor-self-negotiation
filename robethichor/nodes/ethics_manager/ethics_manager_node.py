import rclpy
from rclpy.node import Node

class EthicsManagerNode(Node):
    def __init__(self):
        super().__init__('ethics_manager_node')

def main(args=None):
    rclpy.init(args=args)
    node = EthicsManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()