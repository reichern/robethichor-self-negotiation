import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from robethichor_interfaces.srv import UserStatusService

from robethichor.nodes.context_manager.analyzer import Analyzer
from robethichor.nodes.context_manager.monitor import Monitor

class ContextManagerNode(Node):
    def __init__(self):
        super().__init__('context_manager_node')

        # Subscribers setup
        self.create_subscription(String, 'user_status', self.user_status_update_callback, 10)

        # Publishers setup
        self.current_context_publisher = self.create_publisher(String, 'current_context', 10)

        # Service setup
        self.user_status_service = self.create_service(UserStatusService, 'user_status_service', self.user_status_request_callback)

        # Models initialization
        self.context_model = dict()
        self.user_status = dict()

        # Context managers initialization
        self.analyzer = Analyzer(self, self.context_model, self.current_context_publisher)
        self.monitor = Monitor(self, self.analyzer)


    def user_status_update_callback(self, msg):
        self.get_logger().info("Called user status update")
        self.user_status = json.loads(msg.data)


    def user_status_request_callback(self, request, response):
        self.get_logger().info("Called user status service")
        response.data = json.dumps(self.user_status)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ContextManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()