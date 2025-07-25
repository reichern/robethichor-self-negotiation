import sys
import rclpy
import json
from threading import Event
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool

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
        self.current_active_profile = None # {"disposition": value}
        self.current_user_status = None # {"condition": True/False}
        self.interrupting_user1_active_profile = None # {"disposition": value}
        self.interrupting_user1_status = None # {"condition": True/False}
        self.interrupting_user2_active_profile = None # {"disposition": value}
        self.interrupting_user2_status = None # {"condition": True/False}



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

        self.ethical_impact_analyzer = EthicalImpactAnalyzer()

        # Components initialization
        self.current_offer_generator = OfferGenerator()
        self.interrupting1_offer_generator = OfferGenerator()
        self.interrupting2_offer_generator = OfferGenerator()
        # generators = [self.current_offer_generator, self.interrupting_offer_generator]
        self.current_utility_function = UtilityFunction(ethical_implications, disposition_activation, self)
        self.interrupting1_utility_function = UtilityFunction(ethical_implications, disposition_activation, self)
        self.interrupting2_utility_function = UtilityFunction(ethical_implications, disposition_activation, self)
        # utilities = [self.current_utility_function, self.interrupting_utility_function]

        # Subscriber setup
        self.current_active_profile_subscriber = self.create_subscription(String, 'active_profile', self.current_active_profile_update_callback, 10, callback_group=self.callback_group)
        self.interrupting1_active_profile_subscriber = self.create_subscription(String, 'interrupting_user_1/active_profile', self.interrupting1_active_profile_update_callback, 10, callback_group=self.callback_group)
        self.interrupting2_active_profile_subscriber = self.create_subscription(String, 'interrupting_user_2/active_profile', self.interrupting2_active_profile_update_callback, 10, callback_group=self.callback_group)

        self.data_ready_publisher = self.create_publisher(Bool, 'data_ready', 10)

        # Setup negotiation stuff 
        # .negotiation_publisher = self.create_publisher(String, '/negotiation_msgs', 10)
        # TODO give generators and utilities to negotiation engine
        self.negotiation_engine = NegotiationEngine(self)

        # Negotiation service setup
        self.negotiation_service = self.create_service(NegotiationService, 'negotiation', self.negotiation_service_callback, callback_group=self.callback_group)

        # Client service setup
        self.current_user_status_service_client = self.create_client(UserStatusService, 'user_status_service', callback_group=self.callback_group)
        while not self.current_user_status_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for current_user_status_service to be available')
        self.interrupting_user1_status_service_client = self.create_client(UserStatusService, 'interrupting_user_1/user_status_service', callback_group=self.callback_group)
        self.interrupting_user2_status_service_client = self.create_client(UserStatusService, 'interrupting_user_2/user_status_service', callback_group=self.callback_group)


    def active_profile_update_callback(self, msg, utility_function):
        self.get_logger().info(f"Received new active profile: {msg.data}")
        self.active_profile = json.loads(msg.data)

        # Calculate task ethical impacts and provide them to the utility function using the ethical impact analyzer
        task_ethical_impacts = self.ethical_impact_analyzer.compute_task_ethical_impacts(self.active_profile)
        utility_function.set_task_ethical_impacts(task_ethical_impacts)

    def current_active_profile_update_callback(self,msg):
        self.get_logger().info('setting ethical impacts for current user')
        self.active_profile_update_callback(msg,self.current_utility_function)

    def interrupting1_active_profile_update_callback(self,msg):
        self.get_logger().info('setting ethical impacts for interrupting user 1')
        self.active_profile_update_callback(msg,self.interrupting1_utility_function)
        ready = Bool()
        ready.data = True
        self.data_ready_publisher.publish(ready)

    def interrupting2_active_profile_update_callback(self,msg):
        self.get_logger().info('setting ethical impacts for interrupting user 2')
        self.active_profile_update_callback(msg,self.interrupting2_utility_function)
        ready = Bool()
        ready.data = True
        self.data_ready_publisher.publish(ready)


    def get_user_services(self, user):
        if user == "current":
            return self.current_user_status_service_client, self.current_user_status_service_callback, self.current_offer_generator, self.current_utility_function
        if user == "interrupting_1":
            return self.interrupting_user1_status_service_client, self.interrupting_user1_status_service_callback, self.interrupting1_offer_generator, self.interrupting1_utility_function
        if user == "interrupting_2":
            return self.interrupting_user2_status_service_client, self.interrupting_user2_status_service_callback, self.interrupting2_offer_generator, self.interrupting2_utility_function

    def get_user_status(self, user):
        if user == "current":
            return self.current_user_status
        if user == "interrupting_1":
            return self.interrupting_user1_status
        if user == "interrupting_2":
            return self.interrupting_user2_status

    def negotiation_service_callback(self, request, response):

        if not self.negotiating:
            self.negotiating = True
            self.current_user_status = None
            self.interrupting_user1_status = None
            self.interrupting_user2_status = None
            user1 = request.user1
            user2 = request.user2
            self.get_logger().info(f"Starting negotiation between {user1} and {user2}")

            # Get user 1 status
            user1_status_client, user1_status_callback, user1_generator, user1_utility = self.get_user_services(user1)
            while not user1_status_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {user1} user\'s service to be available')
            user1_status_response_event = Event()
            future = user1_status_client.call_async(UserStatusService.Request())
            future.add_done_callback(lambda future: user1_status_callback(future, user1_status_response_event))
            user1_status_response_event.wait(timeout=self.timeout)
            user1_status = self.get_user_status(user1)
            if user1_status is None:
                self.get_logger().info(f"No user status got for {user1}, entering negotiation without active conditions")
                user1_status = {}

            # Get user 2 status
            user2_status_client, user2_status_callback, user2_generator, user2_utility = self.get_user_services(user2)
            while not user2_status_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {user2} user\'s service to be available')
            user2_status_response_event = Event()
            future = user2_status_client.call_async(UserStatusService.Request())
            future.add_done_callback(lambda future: user2_status_callback(future, user2_status_response_event))
            user2_status_response_event.wait(timeout=self.timeout)
            user2_status = self.get_user_status(user2)
            if user2_status is None:
                self.get_logger().info(f"No user status got for {user2}, entering negotiation without active conditions")
                user2_status = {}

            self.negotiation_engine.configure([user1,user2],[user1_generator,user2_generator],[user1_utility, user2_utility])

            # Generate offers
            tasks = request.tasks
            user1_generator.generate_offers(user1_status, tasks)
            user2_generator.generate_offers(user2_status, tasks)
            self.get_logger().info(f"Offers generated: {user1_generator.get_offers()}, {user2_generator.get_offers()}")

            # Start the negotiation engine
            outcome, rounds = self.negotiation_engine.self_negotiation()
            response.outcome = outcome # current, interruption_1, interruption_2 no-agreement
            response.rounds = rounds

            self.get_logger().info(f"Negotiation completed!")

            self.negotiating = False
        else:
            self.get_logger().info("Received a new negotiation request but a negotiation is in progress. Sending back an empty response.")

        return response        

    def current_user_status_service_callback(self, future, event):
        user_status_response = future.result()
        self.get_logger().info(f"Received user status: {user_status_response.data}")
        self.current_user_status = json.loads(user_status_response.data)
        self.current_user_status_service_client.remove_pending_request(future)
        event.set()

    def interrupting_user1_status_service_callback(self, future, event):
        user_status_response = future.result()
        self.get_logger().info(f"Received user status: {user_status_response.data}")
        self.interrupting_user1_status = json.loads(user_status_response.data)
        self.interrupting_user1_status_service_client.remove_pending_request(future)
        event.set()

    def interrupting_user2_status_service_callback(self, future, event):
        user_status_response = future.result()
        self.get_logger().info(f"Received user status: {user_status_response.data}")
        self.interrupting_user2_status = json.loads(user_status_response.data)
        self.interrupting_user2_status_service_client.remove_pending_request(future)
        event.set()

def main(args=None):
    rclpy.init(args=args)
    node = NegotiationManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()