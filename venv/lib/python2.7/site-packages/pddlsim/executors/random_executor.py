# import pddl.parsersimulate
import random
# from pddlsim.successors.successor import Successor


class RandomExecutor(object):

    """ RandomExecutor - pick a random valid action each step
        the trick is finding out the valid actions
        Most of the code here is a python implementation of that
        But using the tracked successor is significantly faster
    """

    def __init__(self, stop_at_goal=True):
        super(RandomExecutor, self).__init__()
        self.stop_at_goal = stop_at_goal
        self.successor = None

    def initialize(self, services):
        self.services = services

    def next_action(self):
        if self.stop_at_goal and self.services.goal_tracking.reached_all_goals():
            return None
        options = self.services.valid_actions.get() + (
            [] if self.stop_at_goal else [None])
        if len(options) == 0:
            return None
        if len(options) == 1:
            return options[0]
        return self.pick_from_many(options)

    def pick_from_many(self, options):
        chosen = random.choice(options)
        return chosen
