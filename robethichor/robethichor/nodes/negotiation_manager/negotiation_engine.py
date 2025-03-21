class NegotiationEngine():
    def __init__(self, node, utilities):
        self.node = node
        self.utility_f_current = utilities[0]
        self.utility_f_interrupting = utilities[1]
        self.offer_current = None
        self.offer_interrupting = None

        self.accept_current = False
        self.accept_interrupting = False

    def set_offers(self,offer_current, offer_interrupting):
        self.offer_current = offer_current
        self.offer_interrupting = offer_interrupting

    def self_negotiation(self):
        # Execute self negotiation

        self.node.get_logger().debug("Starting Self-Negotiation")

        # Evaluate offer from point of view of current user 
        self.node.get_logger().debug("Evaluating offers from point of view of current user")
        self.accept_current = _evaluate_offers(self,self.utility_f_current, self.offer_interrupting, self.offer_current)

        # Evaluate offer from point of view of interrupting user 
        self.node.get_logger().debug("Evaluating offers from point of view of interrupting user")
        self.accept_interrupting = _evaluate_offers(self,self.utility_f_interrupting, self.offer_current, self.offer_interrupting)

        # Make final decision
        self.node.get_logger().debug("Final Decision:")
        if self.accept_current and not self.accept_interrupting:
            self.node.get_logger().debug("Agreement: Interrupting user is winner")
            # TODO
            return "interrupting"
        elif not self.accept_current and self.accept_interrupting:
            self.node.get_logger().debug("Agreement: Current user is winner")
            # TODO
            return "current"
        else:
            self.node.get_logger().debug("No agreement: Robot remains with current user")
            # TODO
            return "no-agreement"

def _evaluate_offers(self, utility_function, offer_1, offer_2):
    utility_value = utility_function.compute_utility(offer_1, offer_2)
    
    if utility_value > 0:
        # Accept opponent's offer
        self.node.get_logger().debug(f"Utility value: {utility_value}: accept")
        return True
    else:
        # Reject opponent's offer
        self.node.get_logger().debug(f"Utility value: {utility_value}: reject ")
        return False
