import json
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

class LifecycleManager():
    def __init__(self, node, namespace):
        self.node = node 
        self.ns = namespace
        self.callback_group = ReentrantCallbackGroup()

        # Initialize change_state_client for lifecycle context manager and ethics manager node
        self.change_context_manager_state = self.node.create_client(ChangeState, f'{self.ns}/context_manager_node/change_state', callback_group=self.callback_group)      
        self.change_ethics_manager_state = self.node.create_client(ChangeState, f'{self.ns}/ethics_manager_node/change_state', callback_group=self.callback_group)

        # setup for switching user data
        self.interrupt_context = None
        self.interrupt_profile = None
        self.interrupt_status = None
        self.interrupting_user_context_listener = self.node.create_subscription(String, f'{self.ns}/current_context', self.context_callback, 10, callback_group=self.callback_group)
        self.interrupting_user_profile_listener = self.node.create_subscription(String, f'{self.ns}/ethic_profile', self.profile_callback, 10, callback_group=self.callback_group)
        self.interrupting_user_status_listener = self.node.create_subscription(String, f'{self.ns}/user_status', self.status_callback, 10, callback_group=self.callback_group)
        self.user_context_pub = self.node.create_publisher(String, 'current_context', 10)        
        self.user_profile_pub = self.node.create_publisher(String, 'ethic_profile', 10)
        self.user_status_pub = self.node.create_publisher(String, 'user_status', 10)


    def activate_lifecycle_nodes(self):
        # Transition to configured -> inactive -> activated state for context manager
        self.transition_state(self.change_context_manager_state,"configure")
        self.transition_state(self.change_context_manager_state,"activate")

        # Transition to configured -> inactive -> activated state for ethics manager
        self.transition_state(self.change_ethics_manager_state,"configure")
        self.transition_state(self.change_ethics_manager_state,"activate")
        self.node.get_logger().info(f"activated lifecycle nodes in ns {self.ns}")
    
    def deactivate_lifecycle_nodes(self):
        # Transition to configured -> inactive -> activated state for context manager
        self.transition_state(self.change_context_manager_state,"deactivate")
        self.transition_state(self.change_context_manager_state,"cleanup")

        # Transition to configured -> inactive -> activated state for ethics manager
        self.transition_state(self.change_ethics_manager_state,"deactivate")
        self.transition_state(self.change_ethics_manager_state,"cleanup")        
        self.node.get_logger().info(f"deactivated lifecycle nodes in ns {self.ns}")

    def context_callback(self,msg):
        self.interrupt_context = json.loads(msg.data)
        self.node.get_logger().info(f"lifecycle manager: updated interrupting user context")

    def profile_callback(self,msg):
        self.interrupt_profile = json.loads(msg.data)
        self.node.get_logger().info(f"lifecycle manager: updated interrupting user profile")

    def status_callback(self,msg):
        self.interrupt_status = json.loads(msg.data)
        self.node.get_logger().info(f"lifecycle manager: updated interrupting user status")

    def publish_data(self,publisher, data):
        message = String()
        message.data = json.dumps(data)
        publisher.publish(message)

    def switch_user_data(self,switch=False):
        self.node.get_logger().info(f"lifecycle manager: switching user data: {switch}")
        # if switch:
        #     self.publish_data(self.user_context_pub,self.interrupt_context)
        #     self.publish_data(self.user_profile_pub,self.interrupt_profile)
        #     self.publish_data(self.user_status_pub,self.interrupt_status)
        #     self.node.get_logger().info(f"transfered interrupting user's data")
        self.interrupt_context = None
        self.interrupt_profile = None
        self.interrupt_status = None

    def transition_state(self, change_state, transition):
        request = ChangeState.Request()
        request.transition = Transition(label=transition)
        future = change_state.call_async(request)
        # rclpy.spin_until_future_complete(self, future)