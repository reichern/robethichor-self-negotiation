from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState

class LifecycleManager():
    def __init__(self, change_context_manager_state, change_ethics_manager_state):
        # Initialize change_state_client for lifecycle context manager and ethics manager node
        self.change_context_manager_state = change_context_manager_state
        self.change_ethics_manager_state = change_ethics_manager_state
        self.request = ChangeState.Request()

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
        self.request.transition = Transition(label=transition)
        future = change_state.call_async(self.request)
        # rclpy.spin_until_future_complete(self, future)