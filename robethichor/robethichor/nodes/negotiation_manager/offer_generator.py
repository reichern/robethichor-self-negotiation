class OfferGenerator():
    def __init__(self):
        self.offers = list() # [(["task"], ["condition"])]

    def generate_offers(self, user_status, tasks): # user_status is {"condition": True/False}, tasks is ["task_name"]
        self.offers = list()
        conditions_list = list()
        for condition in user_status:
            if user_status[condition]:
                conditions_list.append(condition)
                offer = (tasks, conditions_list.copy())
                self.offers.append(offer)
        self.max_offer = self.offers[-1]
        self.offers.reverse()

    def get_offers(self):
        return self.offers

    def get_max_offer(self):
        return self.max_offer

    def has_next(self):
        return len(self.offers) > 0

    def get_next_offer(self):
        return self.offers.pop()
