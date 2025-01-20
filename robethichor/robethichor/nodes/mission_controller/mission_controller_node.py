import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from robethichor_interfaces.srv import NegotiationService

class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')

        self.callback_group = ReentrantCallbackGroup()

        self.mission_running = False

        self.declare_parameter('log_output_file', '.')
        self.log_output_file = self.get_parameter('log_output_file').get_parameter_value().string_value
        self.get_logger().info(f"Setting mission log output file to {self.log_output_file}")

        # Subscribers setup
        self.create_subscription(String, 'goal', self.start_mission_callback, 10, callback_group=self.callback_group)
        #self.create_subscription(Empty, '/start', self.start_mission_callback, 10, callback_group=self.callback_group)

        #self.create_service(NegotiationService, 'negotiation', self.negotiation_service_callback, callback_group=self.callback_group)

        # Client service setup
        self.negotiation_client = self.create_client(NegotiationService, 'negotiation', callback_group=self.callback_group)
        while not self.negotiation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for negotiation service to be available')

    # def set_goal_callback(self, msg):
    #     self.goal = msg.data

    def start_mission_callback(self, msg):
        self.get_logger().info("Received new goal...")

        if not self.mission_running:
            self.goal = msg.data
            self.mission_running = True

            self.get_logger().info(f"Starting mission, goal: [{self.goal}]")

            negotiation_request = NegotiationService.Request()
            negotiation_request.tasks = ["t1"] # Mocking mission plan. The list of tasks should be obtained by a planner.

            self.start_negotiation_time = time.perf_counter() # Measuring negotiation time

            future = self.negotiation_client.call_async(negotiation_request)
            future.add_done_callback(lambda future: self.negotiation_callback(future))

        else:
            self.get_logger().info("A mission is already being executed: rejecting goal.")

    def negotiation_callback(self, future):
        negotiation_response = future.result()
        self.negotiation_client.remove_pending_request(future)

        if negotiation_response is not None:
                self.get_logger().info(f"Negotiation result: {negotiation_response.outcome} ({negotiation_response.rounds} rounds)")
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
                f.write(f"{self.get_namespace()}: negotiation completed. Configuration: {self.goal}. Negotiation time: {negotiation_time:.3f} seconds. Rounds: {negotiation_response.rounds}. Result: {negotiation_response.outcome}\n")
                f.close()

        self.mission_running = False

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