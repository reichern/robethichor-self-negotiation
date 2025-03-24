import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import String, Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from robethichor_interfaces.srv import NegotiationService, InterruptionService

from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node as LNode

class MissionControllerNode(Node): # Mocked version for testing purposes: must be refined/replaced for actual usage
    def __init__(self):
        super().__init__('mission_controller_node')

        self.callback_group = ReentrantCallbackGroup()

        self.mission_running = False

        self.declare_parameter('log_output_file', '.')
        self.log_output_file = self.get_parameter('log_output_file').get_parameter_value().string_value
        self.get_logger().info(f"Setting mission log output file to {self.log_output_file}")

        # Subscribers setup
        self.create_subscription(String, 'goal', self.start_mission_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Bool, 'interrupt', self.interruption_callback, 10, callback_group=self.callback_group)

        # Client service setup
        self.negotiation_client = self.create_client(NegotiationService, 'negotiation', callback_group=self.callback_group)
        while not self.negotiation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for negotiation service to be available')
        self.interruption_client = self.create_client(InterruptionService, 'interruption', callback_group=self.callback_group)
        while not self.interruption_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for interruption service to be available')

    def start_mission_callback(self, msg):
        self.get_logger().info("Received new goal...")

        if not self.mission_running:
            self.goal = msg.data
            self.mission_running = True

            self.get_logger().info(f"Starting mission, goal: [{self.goal}]")

            # REFINEMENT REQUIRED: a planner should be used to generate the list of tasks, and their implementation must be provided

            # TODO: do something!!! 

        else:
            self.get_logger().info("A mission is already being executed: rejecting goal.")

    def interruption_callback(self, msg):
        self.interrupt = msg.data
        if (self.interrupt == True):
            self.get_logger().info(f"Received interrupt!")

            # Interruption request:
            interruption_request = InterruptionService.Request()

            future = self.interruption_client.call_async(interruption_request)
            future.add_done_callback(lambda future: self.interruption_srv_callback(future))

    def negotiation_callback(self, future):
        negotiation_response = future.result()
        self.negotiation_client.remove_pending_request(future)

        if negotiation_response is not None:
                self.get_logger().info(f"Negotiation result: {negotiation_response.outcome}")
        else:
            self.get_logger().error("Negotiation service call failed")

        end_negotiation_time = time.perf_counter() # Measuring negotiation time
        negotiation_time = end_negotiation_time - self.start_negotiation_time
        self.get_logger().info(f"Negotiation time: {negotiation_time:.3f} seconds")

        self.get_logger().info("Mission is completed!")

        if self.log_output_file:
            log_dir = os.path.dirname(self.log_output_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
            with open(self.log_output_file, 'a') as f:
                f.write(f"{self.get_namespace()}: negotiation completed. Configuration: {self.goal}. Negotiation time: {negotiation_time:.3f} seconds. Result: {negotiation_response.outcome}\n")
                f.close()

        self.mission_running = False

    def interruption_srv_callback(self, future):
        interruption_response = future.result()
        self.interruption_client.remove_pending_request(future)
        if interruption_response is None:
            self.get_logger().error("Negotiation service call failed")
        elif interruption_response == True:
            self.get_logger().info("Interruption initialised, negotiation can be started.")
            # Measuring negotiation time
            self.start_negotiation_time = time.perf_counter() 

            # TODO dynamic tasks
            # Negotiation request:
            negotiation_request = NegotiationService.Request()
            negotiation_request.tasks = ["t1"]

            n_future = self.negotiation_client.call_async(negotiation_request)
            n_future.add_done_callback(lambda future: self.negotiation_callback(n_future))
        elif interruption_response == False: 
            self.get_logger().info("Interrupting request rejected, continue current mission. ")

        return

def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()