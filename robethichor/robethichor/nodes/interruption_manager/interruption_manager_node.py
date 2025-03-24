import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from robethichor_interfaces.srv import InterruptionService

class InterruptionManagerNode(Node):
    def __init__(self):
        super().__init__('interruption_manager_node')

        self.callback_group = ReentrantCallbackGroup()

        # Interruption service setup
        self.negotiation_service = self.create_service(InterruptionService, 'interruption', self.interruption_service_callback, callback_group=self.callback_group)



    def interruption_service_callback(self, request, response):
        # TODO check for capabilities
        self.get_logger().info("Checking for necessary robot capabilities")
        response.outcome = True

        # TODO wait for second user's data? 
        self.get_logger().info("Waiting for second user's data")

        # TODO Launch second user's nodes? 
        self.get_logger().info("Launching second user's nodes")

        return response

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

def main(args=None):
    rclpy.init(args=args)
    node = InterruptionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()