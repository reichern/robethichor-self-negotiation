class UtilityFunction():
    def __init__(self, ethical_implications, disposition_activation, node, is_current_user):
        self.ethical_implications = ethical_implications # {"task": ["disposition"]}
        self.disposition_activation = disposition_activation # {"condition": ["disposition"]}
        self.node = node
        self.is_current_user = is_current_user

    
    def set_task_ethical_impacts(self, ethical_impacts):
        self.ethical_impacts = ethical_impacts # {"disposition": value}

    def compute_utility(self, offer_1, offer_2):
        return self.__compute_offer_utility(offer_1) - self.__compute_offer_utility(offer_2)

    def __compute_offer_utility(self, offer):
        # offer is (["task"], ["condition"])
        value = 0
        eot = False
        for task in offer[0]: # For each task in the offer
            if task in self.ethical_implications: # If it is in the implication list
                for disposition in self.ethical_implications[task]: # For each implied disposition
                    for condition in offer[1]: # For each condition held
                        # If it is in the activation list, if it activates that disposition, and if the disposition is assigned with a value
                        if self.is_current_user and condition == "end of task" and eot == False:
                            value += 1
                            eot = True
                            continue
                        if condition in self.disposition_activation and disposition in self.disposition_activation[condition] and disposition in self.ethical_impacts:
                            value += self.ethical_impacts[disposition] # Add the value to the offer total value
        self.node.get_logger().info(f"Evaluating offer {offer}, value: {value}")
        return value
