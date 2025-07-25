import uuid
import json
import random
from threading import Event
from std_msgs.msg import String

class NegotiationEngine():
    def __init__(self, node):
        self.node = node

        # TODO set value
        self.max_rounds = 1000 

    def configure(self,users,offer_generators,utility_functions):
        self.user1 = users[0]
        self.user2 = users[1]

        self.user1_offer_generator = offer_generators[0]
        self.user1_utility_function = utility_functions[0]

        self.user2_offer_generator = offer_generators[1]
        self.user2_utility_function = utility_functions[1]

    def self_negotiation(self):
        # Initialization
        self.user1_quitted = False
        self.user2_quitted = False
        self.round_counter = 0
        self.switch = True
        self.user1_max_offer = self.user1_offer_generator.get_max_offer()
        self.user2_max_offer = self.user2_offer_generator.get_max_offer()

        # default result: no-agreement 
        # in the case of no-agreement, thecurrent user gets precedence, but it is important to log 
        # that this is not based on an agreement, but default behavior! 
        result = "no-agreement"
        

        # start with interrupting user as sender
        sender = self.user2
        receiver = self.user1

        # max rounds as abort criterion 
        while self.round_counter < self.max_rounds:

            # send offer and evaluate from perspective of other user
            self.round_counter = self.round_counter + 1
            round_result = self.negotiation_round(sender, receiver)
            
            # offer was accepted! 
            if round_result == 1:
                result = sender
                break
            # offer was rejected: switch roles (if no user has quitted yet) and keep negotiating
            elif round_result == -1:
                if self.switch: 
                    sender = self.user1 if sender == self.user2 else self.user2
                    receiver = self.user1 if receiver == self.user2 else self.user2
            # sender has no more offers! 
            elif round_result == 0:
                # if both users have quitted: stop negotiation
                if self.user1_quitted and self.user2_quitted:
                    break
                # only one user has quitted: switch one final time
                else:
                    self.switch = False
                    sender = self.user1 if sender == self.user2 else self.user2
                    receiver = self.user1 if receiver == self.user2 else self.user2

        rounds = self.round_counter
        return result, rounds # results: current, interrupting_1, interrupting_2, no-agreement

    def negotiation_round(self, sender, receiver):
        # set offer generator, utility function and max offer according to current role assignment
        sender_offer_generator = self.user2_offer_generator if sender == self.user2 else self.user1_offer_generator
        receiver_utility_function = self.user1_utility_function if receiver == self.user1 else self.user2_utility_function
        receiver_max_offer = self.user1_max_offer if receiver == self.user1 else self.user2_max_offer

        # if there are no more offers: set sender to quitted
        if not sender_offer_generator.has_next():
            self.node.get_logger().info(f"No more offers from {sender} user!")
            if sender == self.user2: self.user2_quitted = True
            elif sender == self.user1: self.user1_quitted = True
            return 0

        # get next sender offer
        sender_offer = sender_offer_generator.get_next_offer()
        self.node.get_logger().info(f"Next sender offer: {sender_offer}")

        # evaluate offer from perspective of receiver
        utility_value = receiver_utility_function.compute_utility(sender_offer, receiver_max_offer)

        if utility_value > 0:
            # Accept opponent's offer
            self.node.get_logger().info(f"Utility value: {utility_value}: accept")
            return 1
        # Reject opponent's offer
        self.node.get_logger().info(f"Utility value: {utility_value}: reject")
        return -1

