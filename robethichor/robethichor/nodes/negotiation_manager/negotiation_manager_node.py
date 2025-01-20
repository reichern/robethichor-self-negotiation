import rclpy
import json
from threading import Event
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from robethichor_interfaces.srv import UserStatusService
from robethichor_interfaces.srv import NegotiationService

from robethichor.nodes.negotiation_manager.offer_generator import OfferGenerator
from robethichor.nodes.negotiation_manager.utility_function import UtilityFunction
from robethichor.nodes.negotiation_manager.negotiation_engine import NegotiationEngine
from robethichor.nodes.negotiation_manager.ethical_impact_analyzer import EthicalImpactAnalyzer

class NegotiationManagerNode(Node):
    def __init__(self):
        super().__init__('negotiation_manager_node')

        self.callback_group = ReentrantCallbackGroup()

        self.timeout = 5

        self.negotiating = False
        self.active_profile = None # {"disposition": value}
        self.current_user_status = None # {"condition": True/False}

        # Parameters
        self.declare_parameter('ethical_implication_file', '.')
        ethical_implication_filename = self.get_parameter('ethical_implication_file').get_parameter_value().string_value
        self.get_logger().info(f"Reading ethical_implication file from {ethical_implication_filename}")
    
        self.declare_parameter('disposition_activation_file', '.')
        disposition_activation_filename = self.get_parameter('disposition_activation_file').get_parameter_value().string_value
        self.get_logger().info(f"Reading disposition_activation file from {disposition_activation_filename}")

        with open(ethical_implication_filename, 'r') as implication_file, open(disposition_activation_filename, 'r') as activation_file:
            ethical_implications = json.load(implication_file)
            disposition_activation = json.load(activation_file)

        # Components initialization
        self.ethical_impact_analyzer = EthicalImpactAnalyzer()
        self.offer_generator = OfferGenerator()
        self.utility_function = UtilityFunction(ethical_implications, disposition_activation, self)

        # Subscriber setup
        self.active_profile_subscriber = self.create_subscription(String, 'active_profile', self.active_profile_update_callback, 10, callback_group=self.callback_group)

        # Setup negotiation stuff 
        self.negotiation_publisher = self.create_publisher(String, '/negotiation_msgs', 10)
        self.negotiation_engine = NegotiationEngine(self, self.offer_generator, self.utility_function, self.negotiation_publisher)
        self.negotiation_subscriber = self.create_subscription(String, '/negotiation_msgs', self.negotiation_engine.receive_msgs_callback, 10, callback_group=self.callback_group)

        # Negotiation service setup
        self.negotiation_service = self.create_service(NegotiationService, 'negotiation', self.negotiation_service_callback, callback_group=self.callback_group)

        # Client service setup
        self.user_status_service_client = self.create_client(UserStatusService, 'user_status_service', callback_group=self.callback_group)
        while not self.user_status_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for user_status_service to be available')


    def active_profile_update_callback(self, msg):
        self.get_logger().info(f"Received new active profile: {msg.data}")
        self.active_profile = json.loads(msg.data)

        # Calculate task ethical impacts and provide them to the utility function using the ethical impact analyzer
        task_ethical_impacts = self.ethical_impact_analyzer.compute_task_ethical_impacts(self.active_profile)
        self.utility_function.set_task_ethical_impacts(task_ethical_impacts)


    def negotiation_service_callback(self, request, response):

        if not self.negotiating:
            self.negotiating = True
            self.current_user_status = None
            self.get_logger().info("Starting negotiation")

            user_status_response_event = Event()
            # Get user status
            future = self.user_status_service_client.call_async(UserStatusService.Request())
            future.add_done_callback(lambda future: self.user_status_service_callback(future, user_status_response_event))

            user_status_response_event.wait(timeout=self.timeout)
            if self.current_user_status is None:
                self.get_logger().info("No user status got, entering negotiation without active conditions")
                self.current_user_status = {}

            # Generate offers
            tasks = request.tasks
            self.offer_generator.generate_offers(self.current_user_status, tasks)
            self.get_logger().info(f"Offers generated: {self.offer_generator.get_offers()}")

            # Start the negotiation engine
            outcome, rounds = self.negotiation_engine.execute_negotiation()
            response.outcome = outcome # winner, loser, no-agreement
            response.rounds = rounds # no. of rounds

            self.get_logger().info("Negotiation completed")

            self.negotiating = False
        else:
            self.get_logger().info("Received a new negotiation request but a negotiation is in progress. Sending back an empty response.")

        return response

    def user_status_service_callback(self, future, event):
        user_status_response = future.result()
        self.get_logger().info(f"Received user status: {user_status_response.data}")
        self.current_user_status = json.loads(user_status_response.data)
        self.user_status_service_client.remove_pending_request(future)
        event.set()


def main(args=None):
    rclpy.init(args=args)
    node = NegotiationManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()