# https://robotics.stackexchange.com/questions/102991/ros-2-how-to-start-and-stop-a-node-from-a-python-script
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
# from threading import Event
from rclpy.executors import MultiThreadedExecutor

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from robethichor_interfaces.srv import NegotiationService, InterruptionService

class InterruptionManagerNode(Node):
    def __init__(self):
        super().__init__('interruption_manager_node')

        self.callback_group = ReentrantCallbackGroup()

        # Interruption service setup
        self.negotiation_service = self.create_service(InterruptionService, 'interruption', self.interruption_service_callback, callback_group=self.callback_group)
        
        # Negotiation service client setup
        self.negotiation_client = self.create_client(NegotiationService, 'negotiation', callback_group=self.callback_group)
        while not self.negotiation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for negotiation service to be available')

        # Initialize change_state_client for lifecycle context manager and ethics manager node
        self.change_context_manager_state = self.create_client(ChangeState, '/interrupting_user/context_manager_node/change_state', callback_group=self.callback_group)      
        self.change_ethics_manager_state = self.create_client(ChangeState, '/interrupting_user/ethics_manager_node/change_state', callback_group=self.callback_group)
        self.request = ChangeState.Request()

    def interruption_service_callback(self, request, response):
        # if robot does not have necessary capabilities, no negotiation is executed
        self.get_logger().info("Checking for necessary robot capabilities")
        if not self.has_capabilities():
            response.capabilities = False
            response.error = False
            response.winner = "current"
            response.time = -1
            return response

        response.capabilities = True

        # Activate second user's data management nodes 
        self.get_logger().info("Launching second user's nodes")
        self.activate_lifecycle_nodes()

        # TODO wait for second user's active profile
        # without gazebo: 5 seconds is enough, with gazebo: 8 seconds needed!
        # try to do it dynamically! if too quick, there will be no ethical impacts set for utility function... 
        self.get_logger().info("Waiting for second user's data")
        time.sleep(8)
        # self.interrupting_user_status_service_client = self.create_client(UserStatusService, 'interrupting_user/user_status_service', callback_group=self.callback_group)
        # while not self.interrupting_user_status_service_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for interrupting_user_status_service to be available')
# 
        # self.user_status = {}
        # future = self.interrupting_user_status_service_client.call_async(UserStatusService.Request())
        # future.add_done_callback(lambda future: self.user_status_service_callback(future))
        # while self.user_status == {}:
        #     continue

        # Start negotiation
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

        self.deactivate_lifecycle_nodes()

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
    
    def activate_lifecycle_nodes(self):
        # Transition to configured -> inactive -> activated state for context manager
        self.transition_state(self.change_context_manager_state,"configure")
        self.transition_state(self.change_context_manager_state,"activate")

        # Transition to configured -> inactive -> activated state for ethics manager
        self.transition_state(self.change_ethics_manager_state,"configure")
        self.transition_state(self.change_ethics_manager_state,"activate")
    
    def deactivate_lifecycle_nodes(self):
        # Transition to configured -> inactive -> activated state for context manager
        self.transition_state(self.change_context_manager_state,"deactivate")
        self.transition_state(self.change_context_manager_state,"cleanup")

        # Transition to configured -> inactive -> activated state for ethics manager
        self.transition_state(self.change_ethics_manager_state,"deactivate")
        self.transition_state(self.change_ethics_manager_state,"cleanup")

    def transition_state(self, change_state, transition):
        self.request.transition = Transition(label=transition)
        future = change_state.call_async(self.request)
        # rclpy.spin_until_future_complete(self, future)
        
    def user_status_service_callback(self, future):
        self.get_logger().info(f"Received user status: {future.data}")
        self.user_staus = future.data

def main(args=None):
    rclpy.init(args=args)
    node = InterruptionManagerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()