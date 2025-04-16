import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import String, Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from robethichor_interfaces.srv import InterruptionService

class MissionControllerNode(Node): # Mocked version for testing purposes: must be refined/replaced for actual usage
    def __init__(self):
        super().__init__('mission_controller_node')

        self.callback_group = ReentrantCallbackGroup()

        self.mission_running = False
        self.interruption_running = False

        self.declare_parameter('log_output_file', '.')
        self.log_output_file = self.get_parameter('log_output_file').get_parameter_value().string_value
        self.get_logger().info(f"Setting mission log output file to {self.log_output_file}")

        self.declare_parameter('gazebo', rclpy.Parameter.Type.BOOL)
        self.gazebo = self.get_parameter('gazebo').get_parameter_value().bool_value
        self.get_logger().info(f"Start up gazebo: {self.gazebo}")

        # Subscribers setup
        self.create_subscription(String, 'goal', self.start_mission_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, 'interrupting_user/goal', self.interruption_callback, 10, callback_group=self.callback_group)

        # Action Client setup
        if self.gazebo:
            self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            while not self.action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Waiting for navigation action server to be available')

        # Client service setup
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
            # https://github.com/ros2/demos/blob/humble/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_client.py
            if self.gazebo:
                nav_msg = NavigateToPose.Goal()
                nav_msg.pose = self.get_pose(self.goal)

                self.send_goal_future = self.action_client.send_goal_async(nav_msg) # ,feedback_callback=self.feedback_callback)
                
                self.send_goal_future.add_done_callback(self.goal_response_callback)

        else:
            self.get_logger().info("A mission is already being executed: rejecting goal.")

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # TODO read if navigation was successful? 
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

    # def feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger().info('Received feedback: {0}'.format(feedback))

    def interruption_callback(self, msg):
        # TODO what to do if not mission running? what to do if interruption running? 
        if self.mission_running and not self.interruption_running:
            self.interruption_running = True

            self.interrupting_goal = msg.data
            self.get_logger().info(f"Received interrupt! Interrupting goal: {self.interrupting_goal}")

            if self.gazebo:
                # stop current navigation 
                self.goal_handle.cancel_goal_async()

            # Interruption request:
            interruption_request = InterruptionService.Request()
            interruption_request.tasks = ["t1"]

            future = self.interruption_client.call_async(interruption_request)
            future.add_done_callback(lambda future: self.interruption_service_callback(future))

    def interruption_service_callback(self, future):
        interruption_response = future.result()
        self.interruption_client.remove_pending_request(future)
        if interruption_response is None:
            self.get_logger().error("Interruption service call failed, continue current mission.")
        elif interruption_response.capabilities == False:
            self.get_logger().info("Interrupting request rejected because of lacking capabilities, continue current mission. ")
        elif interruption_response.error == True:
            self.get_logger().info("Error occured during negotiation, continue current mission. ")
        else:
            self.get_logger().info("Negotiation successful, Mission is completed!")

            if self.log_output_file:
                log_dir = os.path.dirname(self.log_output_file)
                if log_dir and not os.path.exists(log_dir):
                    os.makedirs(log_dir)
                with open(self.log_output_file, 'a') as f:
                    f.write(f"{self.get_namespace()}: negotiation completed. Configuration: {self.goal}. Negotiation time: {interruption_response.time:.3f} seconds. Result: {interruption_response.winner}\n")
                    f.close()

            # TODO 
            if self.gazebo and (interruption_response.winner == "current" or interruption_response.winner == "no-agreement"):
                nav_msg = NavigateToPose.Goal()
                nav_msg.pose = self.get_pose(self.goal)

                self.send_goal_future = self.action_client.send_goal_async(nav_msg) 
                
                self.send_goal_future.add_done_callback(self.goal_response_callback)
            elif self.gazebo and interruption_response.winner == "interrupting":
                nav_msg = NavigateToPose.Goal()
                nav_msg.pose = self.get_pose(self.interrupting_goal)

                self.send_goal_future = self.action_client.send_goal_async(nav_msg)
                
                self.send_goal_future.add_done_callback(self.goal_response_callback)


            self.mission_running = False
        self.interruption_running = False

    def get_pose(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        if "navigate_to_room_1" in msg:
            pose.pose.position.x = 5.0
            pose.pose.position.y = 2.0
        if "navigate_to_room_2" in msg:
            pose.pose.position.x = 5.0
            pose.pose.position.y = -2.5
        
        return pose



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