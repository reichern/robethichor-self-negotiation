import json
from std_msgs.msg import String

class Executor():
    def __init__(self, node, active_profile, ethic_profiles, active_profile_publisher):
        self.node = node
        self.active_profile = active_profile
        self.ethic_profiles = ethic_profile
        self.active_profile_publisher = active_profile_publisher

    def update_active_profile(self, new_label):
        if new_label in self.ethic_profiles:
            new_profile = self.ethic_profiles[new_label]
            self.active_profile["label"] = new_label
            self.active_profile["profile"] = new_profile

            self.node.get_logger.info(f"Active profile updated")

            message = String()
            message.data = json.dumps(new_profile)
            self.active_profile_publisher.publish(message)
        else:
            self.node.get_logger.info(f"Profile with label {new_label} not found!")
