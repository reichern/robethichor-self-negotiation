import json
from std_msgs.msg import String

class Analyzer():
    def __init__(self, node, context_model, current_context_publisher):
        self.node = node
        self.context_model = context_model
        self.current_context_publisher = current_context_publisher

    def analyze(self, context_update):
        context_changed = False

        for property in context_update:
            if not property in self.context_model or self.context_model[property] != context_update[property]:
                self.node.get_logger().info(f"Updating property {property} to value {context_update[property]}: context has changed")
                self.context_model[property] = context_update[property]
                context_changed = True

        if context_changed:
            message = String()
            message.data = json.dumps(self.context_model)
            self.node.get_logger().info("Publishing new context")
            self.current_context_publisher.publish(message)