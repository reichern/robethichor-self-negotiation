import rclpy
from rclpy.node import Node
from flask import Flask, request, jsonify, make_response

app = Flask(__name__)

class ConnectorNode(Node):
    def __init__(self):
        super().__init__('connector_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5000)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # Flask controller
        app.config['ros_node'] = self
        app.run(host=self.host, port=self.port)

@app.route('/setGoal', methods=['GET'])
def set_goal_controller():
    app.config['ros_node'].get_logger().info("Received setGoal request")
    return make_response(jsonify({}), 201)


def main(args=None):
    rclpy.init(args=args)
    node = ConnectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()