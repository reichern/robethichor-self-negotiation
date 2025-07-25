import json
import rclpy
import time
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
        # Setup publishers for interrupting user 1
        self.int1_ethic_profile_publisher = self.create_publisher(String, 'interrupting_user_1/ethic_profile', 10)
        self.int1_user_status_publisher = self.create_publisher(String, 'interrupting_user_1/user_status', 10)
        self.int1_goal_publisher = self.create_publisher(String, 'interrupting_user_1/goal', 10)
        self.int1_context_publisher = self.create_publisher(String, 'interrupting_user_1/current_context', 10)
        # Setup publishers for interrupting user 2
        self.int2_ethic_profile_publisher = self.create_publisher(String, 'interrupting_user_2/ethic_profile', 10)
        self.int2_user_status_publisher = self.create_publisher(String, 'interrupting_user_2/user_status', 10)
        self.int2_goal_publisher = self.create_publisher(String, 'interrupting_user_2/goal', 10)
        self.int2_context_publisher = self.create_publisher(String, 'interrupting_user_2/current_context', 10)

        # Controller parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5000)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # Flask controller run
        app.config['ros_node'] = self
        app.run(host=self.host, port=self.port)

def publish_data(publisher, subscribers = 1):
    data = request.get_json()
    message = String()
    message.data = json.dumps(data)
    wait_time_start = time.perf_counter()
    while publisher.get_subscription_count() < subscribers:
        if time.perf_counter() - wait_time_start == 10:
            break
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

@app.route('/interrupting_1/profile', methods=['POST'])
def load_int_user_profile():
    node = app.config['ros_node']
    node.get_logger().info("Received interrupting user profile")
    return publish_data(node.int1_ethic_profile_publisher,2)

@app.route('/interrupting_1/status', methods=['POST'])
def set_int_user_status_controller():
    node = app.config['ros_node']
    node.get_logger().info(f"Received interrupting user status")
    return publish_data(node.int1_user_status_publisher,2)

@app.route('/interrupting_1/goal', methods=['POST'])
def set_int_goal_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received interrupting user's setGoal request")
    return publish_data(node.int1_goal_publisher)

@app.route('/interrupting_1/context', methods=['POST'])
def set_int_context_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received interrupting user's base context")
    return publish_data(node.int1_context_publisher,2)

@app.route('/interrupting_2/profile', methods=['POST'])
def load_int2_user_profile():
    node = app.config['ros_node']
    node.get_logger().info("Received second interrupting user profile")
    return publish_data(node.int2_ethic_profile_publisher,2)

@app.route('/interrupting_2/status', methods=['POST'])
def set_int2_user_status_controller():
    node = app.config['ros_node']
    node.get_logger().info(f"Received second interrupting user status")
    return publish_data(node.int2_user_status_publisher,2)

@app.route('/interrupting_2/goal', methods=['POST'])
def set_int2_goal_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received second interrupting user's setGoal request")
    return publish_data(node.int2_goal_publisher)

@app.route('/interrupting_2/context', methods=['POST'])
def set_int2_context_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received second interrupting user's base context")
    return publish_data(node.int2_context_publisher,2)


def main(args=None):
    rclpy.init(args=args)
    node = ConnectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()