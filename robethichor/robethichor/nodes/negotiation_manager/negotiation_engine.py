import uuid
import json
import random
from threading import Event
from std_msgs.msg import String

class NegotiationEngine():
    def __init__(self, node, offer_generator, utility_function, negotiation_publisher):
        self.node = node
        self.offer_generator = offer_generator
        self.utility_function = utility_function
        self.negotiation_publisher = negotiation_publisher

        self.negotiation_id = str(uuid.uuid4())

        self.timeout = 30 # Timeout in seconds for the wait of any message
        self.init_negotiation()

    def init_negotiation(self):
        self.dice_event = Event()
        self.receive_offer_event = Event() # This event covers both the received offer and the quit
        self.receive_decision_event = Event()

        self.my_dice = 0
        self.opponent_dice = 0

        self.quitted = False
        self.opponent_quit_seen = False

        self.opponent_offer = None # (["task"], ["condition"])
        self.opponent_quitted = False
        self.opponent_decision = None

    def receive_msgs_callback(self, msg):
        # Messages are encoded as json strings
        message = json.loads(msg.data)
        id = message["id"]

        if id != self.negotiation_id:
            self.node.get_logger().info("Received Negotiation message!")

            key = message["key"]
            content = message["content"]

            # Keys can be {dice, offer, decision}
            if key == "dice":
                self.opponent_dice = content
                self.dice_event.set()
            elif key == "offer":
                self.opponent_offer = (content['task'], content['conditions'])
                self.receive_offer_event.set()
            elif key == "quit":
                self.opponent_quitted = True
                self.receive_offer_event.set()
            elif key == "decision":
                self.opponent_decision = content
                self.receive_decision_event.set()
            else:
                self.node.get_logger().info("Key not recognized: ignoring message")


    def send_dice(self):
        message = String()
        message.data = json.dumps({"id": self.negotiation_id, "key": "dice", "content": self.my_dice})
        self.negotiation_publisher.publish(message)

    def send_offer(self, offer):
        encoded_offer = {"task": offer[0], "conditions": offer[1]}
        message = String()
        message.data = json.dumps({"id": self.negotiation_id, "key": "offer", "content": encoded_offer})
        self.negotiation_publisher.publish(message)

    def send_quit(self):
        message = String()
        message.data = json.dumps({"id": self.negotiation_id, "key": "quit", "content": True})
        self.negotiation_publisher.publish(message)

    def send_decision(self, decision):
        message = String()
        message.data = json.dumps({"id": self.negotiation_id, "key": "decision", "content": decision})
        self.negotiation_publisher.publish(message)


    def execute_negotiation(self):
        # PRE-NEGOTIATION HAND SHAKING
        # 1- Select a number
        # 2- Publish the number to the channel
        # 3- Subscribe to the channel and wait for the other's number to be received
        # 4- Select the maximum offer to be used for evaluation
        # 5- Decide if sender or receiver

        # 1
        self.my_dice = random.randint(1, 100000)
        self.node.get_logger().info(f"Negotiation: extracted dice value is {self.my_dice}")

        # 2
        self.send_dice()

        # 3
        ## Running spin_once causes the thread to lock
        # w_time = 0
        # while self.opponent_dice == 0 and w_time < self.timeout:
        #     self.node.get_logger().info("Waiting!")
        #     rclpy.spin_once(self.node, timeout_sec=1.0)
        #     w_time += 1
        # if w_time >= self.timeout:
        #     self.node.get_logger().info("Negotiation: no dice received, executing exiting negotiation as winner")
        #     self.init_negotiation()
        #     return "winner"

        self.dice_event.wait(self.timeout)
        self.dice_event.clear()
        if self.opponent_dice == 0:
            self.node.get_logger().info("Negotiation: no dice received, executing exiting negotiation as winner")
            self.init_negotiation()
            return "winner"

        # 4
        self.max_offer = self.offer_generator.get_max_offer()
        self.node.get_logger().info(f"Max offer: {self.max_offer}")

        # 5
        if self.my_dice > self.opponent_dice:
            self.node.get_logger().info("Start the negotiation as sender")
            result = self.sender_negotiation()
        else:
            self.node.get_logger().info("Start the negotiation as receiver")
            result = self.receiver_negotiation()

        # When a negotiation finishes, reinit it
        self.init_negotiation()
        return result # results: winner, loser, no-agreement

    def sender_negotiation(self):
        # IF SENDER
        # 1- Select the offer
        # 2- Send the offer
        # 3- Wait for the response
        # 4a- If the response is accept, exit from the negotiation and return the control
        # 4b- If the response is skip (because the sender quitted), switch role and become receiver
        # 4c- If the response is reject, switch role and become receiver

        # Checking exit condition:
        if not self.offer_generator.has_next(): # If there are no more offers
            if self.quitted: # If the agent already sent the quit message
                if self.opponent_quitted: # And the opponent quitted as well
                    return "no-agreement"
                else:
                    self.node.get_logger().info("Already quitted: skipping sender role")
                    return self.receiver_negotiation()

            else:
                self.node.get_logger().info("No other offers to send: sending quit message")
                self.quitted = True
                self.send_quit()

        else:
            self.node.get_logger().info("Running the negotiation as sender")

            # 1
            offer = self.offer_generator.get_next_offer()
            self.node.get_logger().info(f"Sending offer: {offer}")

            # 2
            self.send_offer(offer)

        # 3
        ## Running spin_once causes the thread to lock
        # w_time = 0
        # while (self.opponent_decision is None) and w_time < self.timeout:
        #     rclpy.spin_once(self.node, timeout_sec=1.0)
        #     w_time += 1
        # if w_time >= self.timeout:
        #     self.node.get_logger().info(f"No decision received: exiting negotiation as winner")
        #     return "winner" # If the opponent does not decide, execute the offered behavior

        self.receive_decision_event.wait(self.timeout)
        self.receive_decision_event.clear()
        if self.opponent_decision is None:
            self.node.get_logger().info("No decision received: exiting negotiation as winner")
            return "winner"

        # 4a
        if self.opponent_decision == 'accept':
            self.node.get_logger().info(f"Opponent accepted the offer: exiting negotiation as winner")
            return "winner"
        # 4b
        elif self.opponent_decision == 'skip':
            self.node.get_logger().info("Opponent skipped over the quit message: switching role")
            self.opponent_decision = None
            return self.receiver_negotiation()
        # 4c
        else:
            # Change role and reset opponent decision and 
            self.node.get_logger().info(f"Opponent rejected: switching role")
            self.opponent_decision = None
            return self.receiver_negotiation()

    def receiver_negotiation(self):
        # IF RECEIVER
        # 1- Wait for the offer to be received
        # 2- Involve the task evaluator and evaluate the received offer
        # 3- Compare the returned value and take decision
        # 4- Send the decision
        # 5a- If the decision is accept, exit from the negotiation and return the control
        # 5b- If the decision is reject, switch role and become sender

        if self.opponent_quit_seen:
            self.node.get_logger().info("Skipping receiver role since the opponent already quitted and will skip its sender turns")
            return self.sender_negotiation()

        self.node.get_logger().info("Running the negotiation as receiver")

        # 1
        ## Running spin_once causes the thread to lock
        # w_time = 0
        # while (self.opponent_offer is None or not self.opponent_quitted) and w_time < self.timeout:
        #     rclpy.spin_once(self.node, timeout_sec=1.0)
        #     w_time += 1
        # if w_time >= self.timeout:
        #     self.node.get_logger().info(f"No offer received: exiting negotiation as winner")
        #     return 'winner'

        self.receive_offer_event.wait(timeout=self.timeout)
        self.receive_offer_event.clear()

        # If nothing new has been received
        if self.opponent_offer is None and not self.opponent_quitted:
            self.node.get_logger().info("No offer received: exiting negotiation as winner")
            return "winner"

        # If instead of an offer, the opponent decided to quit...
        if self.opponent_quitted:
            self.opponent_quit_seen = True
            if self.quitted:
                self.node.get_logger().info("Also opponent quitted: exit without agreement")
                return "no-agreement"

            self.node.get_logger().info("Opponent quitted: skipping")
            self.quit_seen = True
            self.send_decision('skip') # Send skip to unlock the opponent and allow it to receive the next offer
            return self.sender_negotiation()

        self.node.get_logger().info(f"Received offer {self.opponent_offer}")

        # 3
        utility_value = self.utility_function.compute_utility(self.opponent_offer, self.max_offer)

        # 4
        if utility_value > 0:
            # Accept opponent's offer
            self.node.get_logger().info(f"Utility value: {utility_value}: accept")
            # 5
            self.send_decision('accept')
            # 6a
            return 'loser'
        else:
            # Reject opponent's offer, reset received offers and switch role
            self.node.get_logger().info(f"Utility value: {utility_value}: reject and switch roles")
            # 5
            self.send_decision('reject')
            # 6b
            self.opponent_offer = None
            return self.sender_negotiation()
