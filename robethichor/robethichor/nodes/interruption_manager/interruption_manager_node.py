import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
# from threading import Event
from rclpy.executors import MultiThreadedExecutor

from robethichor_interfaces.srv import NegotiationService, InterruptionService

from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node as LNode

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

        # TODO wait for second user's data? 
        self.get_logger().info("Waiting for second user's data")

        # TODO Launch second user's nodes? 
        self.get_logger().info("Launching second user's nodes")

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

        # processing results
        if negotiation_response is None:
            self.get_logger().error("Negotiation service call failed")
            response.error = True
            response.winner = "current"
        else:
            self.get_logger().info(f"Negotiation result: {negotiation_response.outcome}")
            response.error = False
            response.winner = negotiation_response.outcome

        # TODO shutdown of second user's data!
        return response
        
    def has_capabilities(self):
        # TODO 
        return True

def main(args=None):
    rclpy.init(args=args)
    node = InterruptionManagerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        # namespace = 'interrupting_user'

        # ld = LaunchDescription([
        #     
        #     GroupAction([
        #         PushRosNamespace(namespace),
        #         LNode(
        #             package='robethichor',
        #             executable='ethics_manager_node',
        #             name='ethics_manager_node'
        #         ),
        #         LNode(
        #             package='robethichor',
        #             executable='context_manager_node',
        #             name='context_manager_node'
        #         ),
        #     ]),
        # ])
        # ls = LaunchService()
        # ls.include_launch_description(ld)
        # ls.run_async()