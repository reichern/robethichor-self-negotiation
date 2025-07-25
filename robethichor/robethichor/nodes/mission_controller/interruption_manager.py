# https://robotics.stackexchange.com/questions/102991/ros-2-how-to-start-and-stop-a-node-from-a-python-script
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool

from robethichor_interfaces.srv import NegotiationService, InterruptionService, UserStatusService
from robethichor.nodes.mission_controller.lifecycle_manager import LifecycleManager

class InterruptionManager():
    def __init__(self, node):

        self.node = node

        self.callback_group = ReentrantCallbackGroup()
        self.timeout = 5

        # subscription to monitor data arriving at ethics manager
        self.data_ready_subscription = self.node.create_subscription(Bool, 'data_ready', self.data_ready_callback, 10)
        self.ethics_ready = False
        
        # Winner publisher setup - only for Rviz visualization! 
        self.negotiation_result_publisher = self.node.create_publisher(String, 'negotiation_result', 10)

        # Negotiation service client setup
        self.negotiation_client = self.node.create_client(NegotiationService, 'negotiation', callback_group=self.callback_group)
        while not self.negotiation_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for negotiation service to be available')

        self.lifecycle_manager_1 = LifecycleManager(node,"/interrupting_user_1")
        self.lifecycle_manager_2 = LifecycleManager(node,"/interrupting_user_2")

        self.second_interrupting_goal = None


    def handle_interruption(self, tasks):
        self.start_preparation_time = time.perf_counter() 

        # if robot does not have necessary capabilities, no negotiation is executed
        self.node.get_logger().info("Checking for necessary robot capabilities.")
        if not self.has_capabilities():
            winner = "current"
            message = "Interrupting request rejected because of lacking capabilities, continue current mission.\n"
            self.publish_result(message)
            return winner, message
        
        # Activate second user's data management nodes 
        self.node.get_logger().info("Launching second user's nodes")
        self.lifecycle_manager_1.activate_lifecycle_nodes()

        # Check whether nodes have launched properly
        if not self.interrupting_nodes_available():
            winner = "error"
            message = "Did not receive interrupting user data within 10 seconds, aborting negotiation and continue current mission.\n"
            self.publish_result(message, error=True)
            return winner, message
        self.ethics_ready = False
        self.end_preparation_time = time.perf_counter() 
        
        # Start negotiation
        self.node.get_logger().info("Interruption initialised, negotiation can be started.")

        negotiation_response, negotiation_time = self.negotiation(tasks,"current","interrupting_1")

        preparation_time = self.end_preparation_time - self.start_preparation_time
        self.node.get_logger().info(f"Preparation time: {preparation_time:.3f} Negotiation time: {negotiation_time:.3f} seconds")

        if negotiation_response is None:
            winner = "error"
            message = "Negotiation service call failed, continue current mission.\n"
            self.publish_result(message, error=True)
            # deactivate second user's data management nodes!
            self.lifecycle_manager_1.deactivate_lifecycle_nodes()
            self.lifecycle_manager_1.switch_user_data(switch=False)
            return winner, message
        else:
            # negotiation was successful
            outcome = negotiation_response.outcome
            rounds = negotiation_response.rounds

            # multi-lateral negotiation (if second interrupt)
            if not (self.second_interrupting_goal == None):
                outcome, prep_time_2, neg_time_2, neg_time_3 = self.multi_lateral_negotiation(outcome)
                preparation_time += prep_time_2
                negotiation_time += neg_time_2
                negotiation_time += neg_time_3

            # deactivate second user's data management nodes!
            self.lifecycle_manager_1.deactivate_lifecycle_nodes()
            self.lifecycle_manager_1.switch_user_data(switch=outcome == "interrupting_1")

            # processing results
            self.node.get_logger().info(f"Negotiation result: {outcome}")
            rounds = rounds
        
            # no agreement: continue current mission
            if outcome == "no-agreement":
                message = "No-agreement reached! Continue current mission."
            # interrupting user wins: set interrupting goal as new goal! 
            elif outcome == "interrupting_1":
                message = "Interrupting user 1 gets precedence - changing to new mission!"
            elif outcome == "interrupting_2":
                message = "Interrupting user 2 gets precedence - changing to new mission!"
            else:
                message = "Current user gets precedence, continue current mission."
            
            self.publish_result(message)

            log_message = f"Multi-lateral: {not self.second_interrupting_goal==None}. Gazebo: {self.node.gazebo}. Negotiation rounds: {rounds}. Preparation time: {preparation_time:.3f} seconds. Negotiation time: {negotiation_time:.3f} seconds. Result: {outcome}\n"
            
            if "Scalability test!" in self.node.goal:
                log_message = log_message[:-1] + ". Goal: " + self.node.goal + "\n"
            else:
                users = "Users: " + self.node.goal[-4] + self.node.interrupting_goal[-4] + ". "
                log_message = users + log_message
            self.second_interrupting_goal = None
            return outcome, log_message
                
    def negotiation(self,tasks,user1,user2):
        # Measuring negotiation time
        self.start_negotiation_time = time.perf_counter() 

        # TODO dynamic tasks
        # Negotiation request:
        negotiation_request = NegotiationService.Request()
        negotiation_request.tasks = tasks
        negotiation_request.user1 = user1
        negotiation_request.user2 = user2
        negotiation_response = self.negotiation_client.call(negotiation_request)

        # processing time
        self.end_negotiation_time = time.perf_counter() # Measuring negotiation time
        negotiation_time = self.end_negotiation_time - self.start_negotiation_time
        return negotiation_response, negotiation_time

    def multi_lateral_negotiation(self, result_AB):
        # Activate third user's data management nodes 
        self.node.get_logger().info("Going into multi-lateral negotiation! Launching third user's nodes")
        self.start_preparation_time_2 = time.perf_counter() 
        self.lifecycle_manager_2.activate_lifecycle_nodes()

        # Check whether nodes have launched properly
        if not self.interrupting_nodes_available():
            message = "Did not receive second interrupting user data within 10 seconds, returning result of first negotiation!\n"
            self.publish_result(message, error=True)
            return result_AB, message
        self.end_preparation_time_2 = time.perf_counter() 
        self.ethics_ready = False 
        
        # Start negotiation
        self.node.get_logger().info("Second interruption initialised, negotiation between current user and second interrupting user can be started.")

        negotiation_response, negotiation_time_2 = self.negotiation(self.second_interrupting_goal,"current","interrupting_2")

        preparation_time_2 = self.end_preparation_time_2 - self.start_preparation_time_2
        self.node.get_logger().info(f"Preparation time: {preparation_time_2:.3f} Negotiation time: {negotiation_time_2:.3f} seconds")
        if negotiation_response is None:
            message = "Negotiation service call with second interruption failed, returning result of first negotiation!\n"
            self.publish_result(message, error=True)
            return result_AB, message

        result_AC = negotiation_response.outcome
        negotiation_time_3 = 0
        if result_AB == "no-agreement":
            result = result_AC
        elif result_AC == "no-agreement":
            result = result_AB
        elif result_AB == "current" and result_AC == "current":
            result = "current"
        else:
            self.node.get_logger().info("Need to start negotiation between interrupting users!")

            negotiation_response, negotiation_time_3 = self.negotiation(self.second_interrupting_goal,"interrupting_1","interrupting_2")

            self.node.get_logger().info(f"Preparation time: -- Negotiation time: {negotiation_time_3:.3f} seconds")
            if negotiation_response is None:
                message = "Negotiation service call between interrupting users failed, returning without agreement!\n"
                self.publish_result(message, error=True)
                return "no-agreement", message

            result_BC = negotiation_response.outcome

            if result_AB == "interrupting_1" and result_AC == "interrupting_2":
                result = result_BC
            elif result_AB == "interrupting_1" and result_AC == "current":
                if result_BC == "interrupting_1": result =  "interrupting_1"
                else: result =  "no-agreement"
            elif result_AB == "current" and result_AC == "interrupting_2":
                if result_BC == "interrupting_2": result =  "interrupting_2"
                else: result = "no-agreement"

        self.lifecycle_manager_2.deactivate_lifecycle_nodes()
        self.lifecycle_manager_2.switch_user_data(switch=result == "interrupting_2")
        return result, preparation_time_2, negotiation_time_2, negotiation_time_3

    def has_capabilities(self):
        # TODO 
        return True
    
    def publish_result(self, message, error=False):
        # log message
        if error:
            self.node.get_logger().error(message)
        else:
            self.node.get_logger().info(message)

        # publish message for rviz
        rviz_msg = String()
        rviz_msg.data = message
        self.negotiation_result_publisher.publish(rviz_msg)

    def interrupting_nodes_available(self):
        # self.interrupting_user_status_service_client = self.create_client(UserStatusService, 'interrupting_user/user_status_service', callback_group=self.callback_group)
        # while not self.interrupting_user_status_service_client.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('Waiting for interrupting_user_status_service to be available')
        
        # Wait for second user's active profile
        # user_status = {}        
        for _ in range(0,10,1):
            self.node.get_logger().info(f"Waiting for second user's data.")
            # if user_status == {}:
            #     result = self.interrupting_user_status_service_client.call(UserStatusService.Request())
            #     user_status = json.loads(result.data)
            if self.ethics_ready == True: # user_status == {} or 
                break
            time.sleep(1)

        if  self.ethics_ready == False: # user_status == {} or
            return False
        else:
            if self.node.gazebo:
                time.sleep(2)
            return True


    def data_ready_callback(self, future):
        if future.data == True:
            self.node.get_logger().info(f"Received signal that interrupting users data is ready!")
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