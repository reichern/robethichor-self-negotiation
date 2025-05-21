from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.callback_groups import ReentrantCallbackGroup

class LifecycleManager():
    def __init__(self, node):
        self.node = node 
        self.callback_group = ReentrantCallbackGroup()

        # Initialize change_state_client for lifecycle context manager and ethics manager node
        self.change_context_manager_state = self.node.create_client(ChangeState, '/interrupting_user/context_manager_node/change_state', callback_group=self.callback_group)      
        self.change_ethics_manager_state = self.node.create_client(ChangeState, '/interrupting_user/ethics_manager_node/change_state', callback_group=self.callback_group)

    def activate_lifecycle_nodes(self):
        # Transition to configured -> inactive -> activated state for context manager
        self.transition_state(self.change_context_manager_state,"configure")
        self.transition_state(self.change_context_manager_state,"activate")

        # Transition to configured -> inactive -> activated state for ethics manager
        self.transition_state(self.change_ethics_manager_state,"configure")
        self.transition_state(self.change_ethics_manager_state,"activate")
    
    def deactivate_lifecycle_nodes(self):
        # Transition to configured -> inactive -> activated state for context manager
        self.transition_state(self.change_context_manager_state,"deactivate")
        self.transition_state(self.change_context_manager_state,"cleanup")

        # Transition to configured -> inactive -> activated state for ethics manager
        self.transition_state(self.change_ethics_manager_state,"deactivate")
        self.transition_state(self.change_ethics_manager_state,"cleanup")

    def transition_state(self, change_state, transition):
        request = ChangeState.Request()
        request.transition = Transition(label=transition)
        future = change_state.call_async(request)
        # rclpy.spin_until_future_complete(self, future)