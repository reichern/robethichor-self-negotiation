import os
import time
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import String, Bool
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from robethichor.nodes.mission_controller.interruption_manager import InterruptionManager

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

        # Interruption Manager
        self.interruption_manager = InterruptionManager(self)

        # Subscribers setup
        self.create_subscription(String, 'goal', self.start_mission_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, 'interrupting_user/goal', self.interruption_callback, 10, callback_group=self.callback_group)

        # Action Client setup
        if self.gazebo:
            self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            while not self.action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Waiting for navigation action server to be available')

    def start_mission_callback(self, msg):
        self.get_logger().info("Received new goal...")

        if not self.mission_running:
            self.goal = msg.data
            self.mission_running = True

            self.get_logger().info(f"Starting mission, goal: [{self.goal}]")

            # TODO REFINEMENT REQUIRED: a planner should be used to generate the list of tasks, and their implementation must be provided

            # https://github.com/ros2/demos/blob/humble/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_client.py
            if self.gazebo:
                nav_msg = NavigateToPose.Goal()
                nav_msg.pose = self.get_pose(self.goal)

                self.send_goal_future = self.action_client.send_goal_async(nav_msg) # ,feedback_callback=self.feedback_callback)
                
                self.send_goal_future.add_done_callback(self.goal_response_callback)

        else:
            self.get_logger().info(f"A mission is already being executed: rejecting goal {msg.data}.")

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # TODO activate currently active user's lifecycle nodes here! 

        self._get_result_future = self.goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_navigation_result_callback)

    def get_navigation_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.mission_running = False
        # TODO deactivate currently active user's lifecycle nodes

    def interruption_callback(self, msg):
        if self.mission_running and not self.interruption_running:
            self.interruption_running = True

            self.interrupting_goal = msg.data
            self.get_logger().info(f"Received interrupt! Interrupting goal: {self.interrupting_goal}")

            if self.gazebo:
                # stop current navigation 
                self.goal_handle.cancel_goal_async()

    	    # TODO dynamic goals!! be self.goal, self.interrupting_goal instead of t1,t2
            winner, log_message = self.interruption_manager.handle_interruption("t1","t2")

            if winner == "interrupting":
                self.goal = self.interrupting_goal
            self.interrupting_goal = None

            # log results
            if self.log_output_file:
                log_dir = os.path.dirname(self.log_output_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
            with open(self.log_output_file, 'a') as f:
                f.write(log_message)
                f.close()

            # send new navigation goal
            if self.gazebo:
                self.send_navigate_goal()
            else:
                # if gazebo is not running, we have no indication when the mission is finished, so we have to stop it here! 
                self.mission_running = False
                
            self.interruption_running = False

    def send_navigate_goal(self):
        nav_msg = NavigateToPose.Goal()
        nav_msg.pose = self.get_pose(self.goal)
        self.send_goal_future = self.action_client.send_goal_async(nav_msg) 
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info("Sent navigation goal!")


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

        if "navigate_to_room_101" in msg:
            pose.pose.position.x = 5.0
            pose.pose.position.y = 2.0
        if "navigate_to_room_102" in msg:
            pose.pose.position.x = 5.0
            pose.pose.position.y = -2.5
        
        return pose    

def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()