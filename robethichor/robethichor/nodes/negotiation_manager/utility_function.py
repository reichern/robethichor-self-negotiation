class UtilityFunction():
    def __init__(self, ethical_implications, disposition_activation, node, is_current_user):
        self.ethical_implications = ethical_implications # {"goal": ["disposition"]}
        self.disposition_activation = disposition_activation # {"condition": ["disposition"]}
        self.node = node
        self.is_current_user = is_current_user

    
    def set_goal_ethical_impacts(self, ethical_impacts):
        self.ethical_impacts = ethical_impacts # {"disposition": value}

    def compute_utility(self, offer_1, offer_2):
        return self.__compute_offer_utility(offer_1) - self.__compute_offer_utility(offer_2)

    def __compute_offer_utility(self, offer):
        # offer is ("goal", ["condition"])
        value = 0
        eog = False # end of goal
        goal =  offer[0] 
        if goal in self.ethical_implications: # If it is in the implication list
            for disposition in self.ethical_implications[goal]: # For each implied disposition
                for condition in offer[1]: # For each condition held
                    # If it specifies that the user is at the end of their goal, add a value of 1
                    if self.is_current_user and condition == "end of goal" and eog == False:
                        value += 1
                        eog = True
                        continue
                    # If it is in the activation list, if it activates that disposition, and if the disposition is assigned with a value
                    if condition in self.disposition_activation and disposition in self.disposition_activation[condition] and disposition in self.ethical_impacts:
                        value += self.ethical_impacts[disposition] # Add the value to the offer total value
        self.node.get_logger().info(f"Evaluating offer {offer}, value: {value}")
        return value
