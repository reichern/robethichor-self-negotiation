import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from robethichor_interfaces.srv import NegotiationService

class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')

        self.callback_group = ReentrantCallbackGroup()

        self.mission_running = False

        # Subscribers setup
        self.create_subscription(String, 'goal', self.set_goal_callback, 10)

        # Client service setup
        self.negotiation_client = self.create_client(NegotiationService, 'negotiation', callback_group=self.callback_group)
        while not self.negotiation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for negotiation service to be available')

        self.start_service = self.create_service(Empty, 'start', self.start_mission_callback, callback_group=self.callback_group)

    def set_goal_callback(self, msg):
        self.goal = msg.data

    def start_mission_callback(self, request, response):

        if not self.mission_running:
            self.mission_running = True

            self.get_logger().info("Starting mission")

            negotiation_request = NegotiationService.Request()
            negotiation_request.tasks = ["t1"]

            future = self.negotiation_client.call_async(negotiation_request)
            future.add_done_callback(lambda future: self.negotiation_callback(future))

        else:
            self.get_logger().info("A mission is already being executed")

        return response

    def negotiation_callback(self, future):
        negotiation_response = future.result()
        self.mission_running = False
        self.negotiation_client.remove_pending_request(future)

        if negotiation_response is not None:
                self.get_logger().info(f"Negotiation result: {negotiation_response.outcome}")
        else:
            self.get_logger().error("Negotiation service call failed")

        self.get_logger().info("Mission is completed!")

def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    #rclpy.spin(node, executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()