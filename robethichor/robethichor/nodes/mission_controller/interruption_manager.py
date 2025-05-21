# https://robotics.stackexchange.com/questions/102991/ros-2-how-to-start-and-stop-a-node-from-a-python-script
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from robethichor_interfaces.srv import NegotiationService, InterruptionService, UserStatusService
from robethichor.nodes.mission_controller.lifecycle_manager import LifecycleManager

class InterruptionManager():
    def __init__(self, node):

        self.node = node

        self.callback_group = ReentrantCallbackGroup()
        self.timeout = 5

        # subscription to monitor data arriving at ethics manager
        self.active_profile_subscription = self.node.create_subscription(String, 'interrupting_user/active_profile', self.received_active_profile_callback, 10)
        self.ethics_ready = False
        
        # Winner publisher setup - only for Rviz visualization! 
        self.winner_publisher = self.node.create_publisher(String, 'winner', 10)

        # Negotiation service client setup
        self.negotiation_client = self.node.create_client(NegotiationService, 'negotiation', callback_group=self.callback_group)
        while not self.negotiation_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for negotiation service to be available')

        self.lifecycle_manager = LifecycleManager(node)


    def handle_interruption(self, tasks):
        # if robot does not have necessary capabilities, no negotiation is executed
        self.node.get_logger().info("Checking for necessary robot capabilities.")
        if not self.has_capabilities():
            winner = "current"
            message = "Interrupting request rejected because of lacking capabilities, continue current mission."
            self.node.get_logger().info(message)
            return winner, message
        
        # Activate second user's data management nodes 
        self.node.get_logger().info("Launching second user's nodes")
        self.lifecycle_manager.activate_lifecycle_nodes()

        # Check whether nodes have launched properly
        if not self.interrupting_nodes_available():
            winner = "error"
            message = "Did not receive interrupting user data within 10 seconds, aborting negotiation and continue current mission."
            self.node.get_logger().error(message)
            return winner, message


        # Start negotiation
        self.node.get_logger().info("Interruption initialised, negotiation can be started.")
        # Measuring negotiation time
        self.start_negotiation_time = time.perf_counter() 

        # TODO dynamic tasks
        # Negotiation request:
        negotiation_request = NegotiationService.Request()
        negotiation_request.tasks = tasks
        negotiation_response = self.negotiation_client.call(negotiation_request)

        # processing time
        end_negotiation_time = time.perf_counter() # Measuring negotiation time
        negotiation_time = end_negotiation_time - self.start_negotiation_time
        self.node.get_logger().info(f"Negotiation time: {negotiation_time:.3f} seconds")

        # deactivate second user's data management nodes!
        self.lifecycle_manager.deactivate_lifecycle_nodes()

        # processing results
        if negotiation_response is None:
            winner = "error"
            message = "Negotiation service call failed, continue current mission."
            self.node.get_logger().error(message)
            return winner, message
        else:
            # negotiation was successful
            self.node.get_logger().info(f"Negotiation result: {negotiation_response.outcome}")
            winner = negotiation_response.outcome
        
            # no agreement: continue current mission
            if winner == "no-agreement":
                message = "No agreement! Continue current mission."
            # interrupting user wins: set interrupting goal as new goal! 
            elif winner == "interrupting":
                message = "Interrupting user gets precedence - changing to new mission!"
            else:
                message = "Current user gets precedence, continue current mission."
            self.node.get_logger().info(message)

            # publish negotiation result to Rviz
            rviz_msg = String()
            rviz_msg.data = message
            self.winner_publisher.publish(rviz_msg)

            log_message = f"{message} Negotiation time: {negotiation_time:.3f} seconds.\n"
            return winner, log_message
                
    def has_capabilities(self):
        # TODO 
        return True

    def interrupting_nodes_available(self):
        # self.interrupting_user_status_service_client = self.create_client(UserStatusService, 'interrupting_user/user_status_service', callback_group=self.callback_group)
        # while not self.interrupting_user_status_service_client.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('Waiting for interrupting_user_status_service to be available')
        
        # Wait for second user's active profile
        # user_status = {}        
        for _ in range(0,10,1):
            self.node.get_logger().info(f"Waiting for second user's data.")
            time.sleep(1)
            # if user_status == {}:
            #     result = self.interrupting_user_status_service_client.call(UserStatusService.Request())
            #     user_status = json.loads(result.data)
            if self.ethics_ready == True: # user_status == {} or 
                break

        if  self.ethics_ready == False: # user_status == {} or
            return False
        else:
            time.sleep(5)
            return True


    def received_active_profile_callback(self, future):
        self.node.get_logger().info(f"Received active profile")
        self.ethics_ready = True
        

# def main(args=None):
#     rclpy.init(args=args)
#     node = InterruptionManagerNode()
#     executor = MultiThreadedExecutor()
#     rclpy.spin(node, executor)
#     rclpy.shutdown()
# 
# if __name__ == '__main__':
#     main()