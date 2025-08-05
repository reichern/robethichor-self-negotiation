import rclpy
import json
# from rclpy.node import Node
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String
from robethichor_interfaces.srv import UserStatusService

from robethichor.nodes.context_manager.analyzer import Analyzer
from robethichor.nodes.context_manager.monitor import Monitor

class ContextManagerNode(Node):
    def __init__(self):
        super().__init__('context_manager_node')

        # Subscribtion 
        self.user_status_subscription = None

        # Publisher 
        self.current_context_publisher = None

        # Service
        self.user_status_service = None 

        # Models initialization
        self.context_model = dict()
        self.user_status = dict()

        # Context managers
        self.analyzer = None 
        self.monitor = None 

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_configure() is called.")

        # Publisher setup
        self.current_context_publisher = self.create_lifecycle_publisher(String, "current_context", 10)

        # Context managers initialization
        self.analyzer = Analyzer(self, self.context_model, self.current_context_publisher)
        self.monitor = Monitor(self, self.analyzer)

        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_activate() is called.")

        # Subscribers setup
        self.user_status_subscription = self.create_subscription(String, 'user_status', self.user_status_update_callback, 10)

        # Service setup
        self.user_status_service = self.create_service(UserStatusService, 'user_status_service', self.user_status_request_callback)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_deactivate() is called.")
        
        # destroy subscription, client
        self.destroy_subscription(self.user_status_subscription)
        self.destroy_service(self.user_status_service)
        self.destroy_publisher(self.current_context_publisher)
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug('on_cleanup() is called.')

        # destroy publisher
        self.context_model = dict()
        self.user_status = dict()
        # destroy analyzer, monitor
        self.analyzer = None
        self.monitor = None


        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug('on_shutdown() is called.')
        return super().on_shutdown(state)

    def user_status_update_callback(self, msg):
        self.get_logger().info("Called user status update")
        self.user_status = json.loads(msg.data)


    def user_status_request_callback(self, request, response):
        self.get_logger().info("Called user status service")
        response.data = json.dumps(self.user_status)
        return response

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.SingleThreadedExecutor()
    node = ContextManagerNode()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.destroy_node()

if __name__ == '__main__':
    main()