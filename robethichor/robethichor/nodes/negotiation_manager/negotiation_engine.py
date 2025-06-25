import uuid
import json
import random
from threading import Event
from std_msgs.msg import String

class NegotiationEngine():
    def __init__(self, node, offer_generators, utility_functions):
        self.node = node
        self.current_offer_generator = offer_generators[0]
        self.current_utility_function = utility_functions[0]

        self.interrupting_offer_generator = offer_generators[1]
        self.interrupting_utility_function = utility_functions[1]

        # TODO set value
        self.max_rounds = 1000 

    def self_negotiation(self):
        # Initialization
        self.current_quitted = False
        self.interrupting_quitted = False
        self.round_counter = 0
        self.switch = True
        self.current_max_offer = self.current_offer_generator.get_max_offer()
        self.interrupting_max_offer = self.interrupting_offer_generator.get_max_offer()

        # default result: no-agreement 
        # in the case of no-agreement, thecurrent user gets precedence, but it is important to log 
        # that this is not based on an agreement, but default behavior! 
        result = "no-agreement"
        

        # start with interrupting user as sender
        sender = "interrupting"
        receiver = "current"

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
                    sender = "interrupting" if sender == "current" else "current"
                    receiver = "interrupting" if receiver == "current" else "current"
            # sender has no more offers! 
            elif round_result == 0:
                # if both users have quitted: stop negotiation
                if self.current_quitted and self.interrupting_quitted:
                    break
                # only one user has quitted: switch one final time
                else:
                    self.switch = False
                    sender = "interrupting" if sender == "current" else "current"
                    receiver = "interrupting" if receiver == "current" else "current"

        rounds = self.round_counter
        return result, rounds # results: current, interrupting, no-agreement

    def negotiation_round(self, sender, receiver):
        # set offer generator, utility function and max offer according to current role assignment
        sender_offer_generator = self.interrupting_offer_generator if sender == "interrupting" else self.current_offer_generator
        receiver_utility_function = self.current_utility_function if receiver == "current" else self.interrupting_utility_function
        receiver_max_offer = self.current_max_offer if receiver == "current" else self.interrupting_max_offer

        # if there are no more offers: set sender to quitted
        if not sender_offer_generator.has_next():
            self.node.get_logger().info(f"No more offers from {sender} user!")
            if sender == "interrupting": self.interrupting_quitted = True
            elif sender == "current": self.current_quitted = True
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

