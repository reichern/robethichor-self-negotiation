import rclpy
from rclpy.node import Node

class NegotiationManagerNode(Node):
    def __init__(self):
        super().__init__('negotiation_manager_node')

def main(args=None):
    rclpy.init(args=args)
    node = NegotiationManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()