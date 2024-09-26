import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
from robethichor_interfaces.srv import NegotiationService

class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')

        # Subscribers setup
        self.create_subscription(String, 'goal', self.set_goal_callback, 10)

        # Services setup
        self.negotiation_client = self.create_client(NegotiationService, 'negotiation')

        self.start_service = self.create_service(Empty, 'start', self.start_mission_callback)

    def set_goal_callback(self, msg):
        self.goal = msg.data

    def start_mission_callback(self, request, response):
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()