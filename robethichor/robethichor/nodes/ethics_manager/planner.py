import json
from std_msgs.msg import String

class Planner():
    def __init__(self, node, active_profile, executor):

        self.node = node
        self.active_profile = active_profile
        self.executor = executor

    def plan_active_profile_change(self, msg):
        current_context = json.loads(msg.data)

        self.node.get_logger().info(f"Received new context: {msg.data}")

        # Planning active profile change
        if current_context["location"] != active_profile["label"]:
            self.node.get_logger().info(f"Active profile needs to be changed with {current_context['location']}-related profile")
            self.executor.update_active_profile(current_context["location"])
        else:
            self.node.get_logger().info("Active profile does not require change")

