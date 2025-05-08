import rclpy
import json
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

from robethichor.nodes.ethics_manager.planner import Planner
from robethichor.nodes.ethics_manager.executor import Executor


class EthicsManagerNode(Node):
    def __init__(self):
        super().__init__('ethics_manager_node')

        # Models initialization
        self.ethic_profiles = dict()
        self.active_profile = dict()

        # Publishers setup
        self.active_profile_publisher = None

        # Profile update managers
        self.executor_ = None 
        self.planner = None 

        # Subscribers setup
        self.profile_subscription = None
        self.context_subscription = None

    def ethic_profiles_update_callback(self, msg):
        self.get_logger().info("Called ethic profiles update")
        updated_profile = json.loads(msg.data)
        for label in updated_profile:
            self.ethic_profiles[label] = updated_profile[label]

        # If there is a change in the active profile, refresh it
        if "label" in self.active_profile and self.active_profile["label"] in self.ethic_profiles:
            self.get_logger().info("There is a change in the active profile, refreshing it")
            self.executor_.refresh_active_profile()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_configure() is called.")

        # Publisher setup
        self.active_profile_publisher = self.create_lifecycle_publisher(String, 'active_profile', 10)

        # Profile update managers initialization
        self.executor_ = Executor(self, self.active_profile, self.ethic_profiles, self.active_profile_publisher)
        self.planner = Planner(self, self.active_profile, self.executor_)

        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_activate() is called.")

        # Subscribers setup
        self.profile_subscription = self.create_subscription(String, 'ethic_profile', self.ethic_profiles_update_callback, 10)
        self.context_subscription = self.create_subscription(String, 'current_context', self.planner.plan_active_profile_change, 10)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_deactivate() is called.")
        
        # destroy subscription
        self.destroy_subscription(self.profile_subscription)
        self.destroy_subscription(self.context_subscription)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:

        # destroy publisher
        self.destroy_publisher(self.active_profile_publisher)

        # TODO executor und planner zerstören? 

        self.get_logger().debug('on_cleanup() is called.')
        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug('on_shutdown() is called.')
        return super().on_shutdown(state)

def main(args=None):
    rclpy.init(args=args)

    # node = EthicsManagerNode()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

    executor = rclpy.executors.SingleThreadedExecutor()
    node = EthicsManagerNode()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.destroy_node()


if __name__ == '__main__':
    main()