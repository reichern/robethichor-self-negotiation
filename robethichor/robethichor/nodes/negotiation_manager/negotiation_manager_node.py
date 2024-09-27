import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from robethichor_interfaces.srv import UserStatusService
from robethichor_interfaces.srv import NegotiationService

class NegotiationManagerNode(Node):
    def __init__(self):
        super().__init__('negotiation_manager_node')

        # Subscriber setup
        self.active_profile_subscriber = self.create_subscription(String, 'active_profile', self.active_profile_update_callback)

        # Service setup
        self.negotiation_service = self.create_service(NegotiationService, 'negotiate', self.negotiation_service_callback)

        # Client service setup
        self.user_status_service_client = self.create_client(UserStatusService, 'user_status_service')
        while not self.user_status_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for user_status_service to be available')

        self.active_profile = None

    def active_profile_update_callback(msg):
        self.get_logger().info("Received new active profile")
        self.active_profile = json.loads(msg.data)

    def negotiation_service_callback(self, request, response):
        self.get_logger().info("Starting negotiation")

        # Get user status
        self.user_status_service_client.call_async(UserStatusService.Request()).add_done_callback(self.generate_offers)

    def generate_offers(self, future):
        user_status = json.loads(future.result())
        #### TBD

def main(args=None):
    rclpy.init(args=args)
    node = NegotiationManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()