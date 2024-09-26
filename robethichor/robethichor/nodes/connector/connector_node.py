import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify, make_response

app = Flask(__name__)

class ConnectorNode(Node):
    def __init__(self):
        super().__init__('connector_node')

        # Publishers setup
        self.ethic_profile_publisher = self.create_publisher(String, 'ethic_profile', 10)
        self.user_status_publisher = self.create_publisher(String, 'user_status', 10)
        self.goal_publisher = self.create_publisher(String, 'goal', 10)

        # Controller parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5000)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # Flask controller run
        app.config['ros_node'] = self
        app.run(host=self.host, port=self.port)


@app.route('/loadEthicProfile', methods=['POST'])
def load_user_profile():
    node = app.config['ros_node']
    node.get_logger().info("Received user profile")
    data = request.get_json()

    message = String()
    message.data = str(data)
    node.ethic_profile_publisher.publish(message)

    return make_response(jsonify({}), 201)

@app.route('/setUserStatus', methods=['POST'])
def set_user_status_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received user status")

    data = request.get_json()

    message = String()
    message.data = str(data)
    node.user_status_publisher.publish(message)

    return make_response(jsonify({}), 201)

@app.route('/setGoal', methods=['POST'])
def set_goal_controller():
    node = app.config['ros_node']
    node.get_logger().info("Received setGoal request")

    data = request.get_json()

    message = String()
    message.data = str(data)
    node.goal_publisher.publish(message)

    return make_response(jsonify({}), 201)


def main(args=None):
    rclpy.init(args=args)
    node = ConnectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()