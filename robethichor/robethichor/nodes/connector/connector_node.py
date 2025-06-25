import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify, make_response

app = Flask(__name__)

class ConnectorNode(Node):
    def __init__(self):
        super().__init__('connector_node')

        # Setup publishers for currently active user
        self.ethic_profile_publisher = self.create_publisher(String, 'ethic_profile', 10)
        self.user_status_publisher = self.create_publisher(String, 'user_status', 10)
        self.goal_publisher = self.create_publisher(String, 'goal', 10)
        self.context_publisher = self.create_publisher(String, 'current_context', 10)
        # Setup publishers for interrupting user
        self.int_ethic_profile_publisher = self.create_publisher(String, 'interrupting_user/ethic_profile', 10)
        self.int_user_status_publisher = self.create_publisher(String, 'interrupting_user/user_status', 10)
        self.int_goal_publisher = self.create_publisher(String, 'interrupting_user/goal', 10)
        self.int_context_publisher = self.create_publisher(String, 'interrupting_user/current_context', 10)

        # Controller parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5000)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # Flask controller run
        app.config['ros_node'] = self
        app.run(host=self.host, port=self.port)

def publish_data(publisher):
    data = request.get_json()
    message = String()
    message.data = json.dumps(data)
    while publisher.get_subscription_count() < 1:
        continue
    publisher.publish(message)

    return make_response(jsonify({}), 201)

@app.route('/profile', methods=['POST'])
def load_user_profile():
    node = app.config['ros_node']
    node.get_logger().info("Received user profile")
    return publish_data(node.ethic_profile_publisher)

@app.route('/status', methods=['POST'])
def set_user_status_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received user status")
    return publish_data(node.user_status_publisher)

@app.route('/goal', methods=['POST'])
def set_goal_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received setGoal request")
    return publish_data(node.goal_publisher)

@app.route('/context', methods=['POST'])
def set_context_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received base context")
    return publish_data(node.context_publisher)

@app.route('/interrupting/profile', methods=['POST'])
def load_int_user_profile():
    node = app.config['ros_node']
    node.get_logger().info("Received interrupting user profile")
    return publish_data(node.int_ethic_profile_publisher)

@app.route('/interrupting/status', methods=['POST'])
def set_int_user_status_controller():
    node = app.config['ros_node']
    node.get_logger().info(f"Received interrupting user status")
    return publish_data(node.int_user_status_publisher)

@app.route('/interrupting/goal', methods=['POST'])
def set_int_goal_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received interrupting user's setGoal request")
    return publish_data(node.int_goal_publisher)

@app.route('/interrupting/context', methods=['POST'])
def set_int_context_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received interrupting user's base context")
    return publish_data(node.int_context_publisher)

def main(args=None):
    rclpy.init(args=args)
    node = ConnectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()