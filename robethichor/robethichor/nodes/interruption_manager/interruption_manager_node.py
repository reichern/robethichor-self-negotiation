# https://robotics.stackexchange.com/questions/102991/ros-2-how-to-start-and-stop-a-node-from-a-python-script
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import ChangeState
import json
from std_msgs.msg import String

from robethichor_interfaces.srv import NegotiationService, InterruptionService, UserStatusService
from robethichor.nodes.interruption_manager.lifecycle_manager import LifecycleManager

class InterruptionManagerNode(Node):
    def __init__(self):
        super().__init__('interruption_manager_node')

        self.callback_group = ReentrantCallbackGroup()
        self.timeout = 5

        # subscription to monitor data arriving at ethics manager
        self.active_profile_subscription = self.create_subscription(String, 'interrupting_user/active_profile', self.received_active_profile_callback, 10)
        self.ethics_ready = False

        # Interruption service setup
        self.negotiation_service = self.create_service(InterruptionService, 'interruption', self.interruption_service_callback, callback_group=self.callback_group)
        
        # Negotiation service client setup
        self.negotiation_client = self.create_client(NegotiationService, 'negotiation', callback_group=self.callback_group)
        while not self.negotiation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for negotiation service to be available')

        change_context_manager_state = self.create_client(ChangeState, '/interrupting_user/context_manager_node/change_state', callback_group=self.callback_group)      
        change_ethics_manager_state = self.create_client(ChangeState, '/interrupting_user/ethics_manager_node/change_state', callback_group=self.callback_group)
        self.lifecycle_manager = LifecycleManager(change_context_manager_state, change_ethics_manager_state)


    def interruption_service_callback(self, request, response):
        # if robot does not have necessary capabilities, no negotiation is executed
        self.get_logger().info("Checking for necessary robot capabilities")
        if not self.has_capabilities():
            response.capabilities = False
            response.error = False
            response.winner = "current"
            response.time = -1.0
            return response

        response.capabilities = True

        # Activate second user's data management nodes 
        self.get_logger().info("Launching second user's nodes")
        self.lifecycle_manager.activate_lifecycle_nodes()
        self.interrupting_user_status_service_client = self.create_client(UserStatusService, 'interrupting_user/user_status_service', callback_group=self.callback_group)
        while not self.interrupting_user_status_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for interrupting_user_status_service to be available')
        
        # wait for second user's active profile
        user_status = {}        
        for _ in range(0,10,1):
            self.get_logger().info(f"Waiting for second user's data: {user_status}, {self.ethics_ready}")
            time.sleep(1)
            if user_status == {}:
                result = self.interrupting_user_status_service_client.call(UserStatusService.Request())
                user_status = json.loads(result.data)
            if not (user_status == {} or self.ethics_ready == False):
                break

        if user_status == {} or self.ethics_ready == False:
            self.get_logger().info(f"Did not receive interrupting user data within 10 seconds, aborting negotiation")
            response.error = True
            response.winner = "current"
            response.time = -1.0
            return response

        # Start negotiation
        time.sleep(1)
        self.get_logger().info("Interruption initialised, negotiation can be started.")
        # Measuring negotiation time
        self.start_negotiation_time = time.perf_counter() 

        # TODO dynamic tasks
        # Negotiation request:
        negotiation_request = NegotiationService.Request()
        negotiation_request.tasks = request.tasks
        negotiation_response = self.negotiation_client.call(negotiation_request)

        # processing time
        end_negotiation_time = time.perf_counter() # Measuring negotiation time
        negotiation_time = end_negotiation_time - self.start_negotiation_time
        self.get_logger().info(f"Negotiation time: {negotiation_time:.3f} seconds")
        response.time = negotiation_time

        self.lifecycle_manager.deactivate_lifecycle_nodes()

        # processing results
        if negotiation_response is None:
            self.get_logger().error("Negotiation service call failed")
            response.error = True
            response.winner = "current"
        else:
            self.get_logger().info(f"Negotiation result: {negotiation_response.outcome}")
            response.error = False
            response.winner = negotiation_response.outcome

        # deactivate second user's data management nodes!
        return response
        
    def has_capabilities(self):
        # TODO 
        return True

    def received_active_profile_callback(self, future):
        self.get_logger().info(f"Received active profile")
        self.ethics_ready = True
        

def main(args=None):
    rclpy.init(args=args)
    node = InterruptionManagerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()