import json
from std_msgs.msg import String

class Monitor():
    def __init__(self, node, analyzer):

        self.analyzer = analyzer
        self.node = node
        
        # Subscribe to topics for context information getting
        self.node.create_subscription(String, 'monitored_properties', self.monitor_context, 10)        

    def monitor_context(self, msg):
        monitored_properties = json.loads(msg.data)

        self.node.get_logger().debug(f"Received info on monitored properties: {msg.data}")
        # Invoke analyzer to check if context is updated
        self.analyzer.analyze(monitored_properties)
