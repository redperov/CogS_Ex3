
import copy

import time
from services.simulator_mediator import SimulatorMediator
from services.goal_tracking import GoalTracking
from services.perception import Perception
from fd_parser import FDParser, PreconditionFalseError


class Simulator(object):
    """ 
    The simulator is in charge of managing the true state of the world
    This means 3 essential things:
    1. The action loop - this is where changes to the true state can be incepted
    2. Applying the change to the true state
    3. Providing perceptions of the true state to external factors (these perceptions aren't required to be truthful)
    """

    def __init__(self, parser):
        super(Simulator, self).__init__()
        self.check_preconditions = True
        self.parser = parser
        self._state = self.parser.build_first_state()
        self.goal_tracking = GoalTracking(
            self.parser, Perception(lambda: self._state))
        self.report_card = ReportCard()

    def simulate(self, next_action_func):
        self.report_card.start()
        self.action_loop(next_action_func)
        return self.report_card.done(self.goal_tracking.reached_all_goals())

    def action_loop(self, next_action_func):
        while True:
            action = next_action_func()
            if not action or action.lower() == '(reach-goal)':
                return
            try:
                self.parser.apply_action_to_state(
                    action, self._state, self.check_preconditions)
                self.goal_tracking.on_action(action)
                self.report_card.add_action()
            except PreconditionFalseError as e:
                self.report_card.add_failed_action()

    def perceive_state(self):
        self.report_card.add_perception()
        return self.parser.copy_state(self._state)


class ReportCard():
    def __init__(self):
        self.success = False
        self.failed_actions = 0
        self.total_perception_requests = 0
        self.total_actions = 0
        self.total_action_costs = 0
        self.start_time = None
        self.end_time = None
        self.total_time = None

    def add_perception(self):
        self.total_perception_requests += 1

    def add_action(self, cost=1):
        self.total_actions += 1
        self.total_action_costs += cost

    def add_failed_action(self, cost=0):
        self.total_actions += 1
        self.failed_actions += 1
        self.total_action_costs += cost

    def start(self):
        """ 
        Record start time
        :return: self for a fluent api
        """
        self.start_time = time.time()
        return self

    def done(self, success):
        """
        Record end time and success
        :return: self for a fluent api
        """
        self.end_time = time.time()
        self.total_time = self.end_time - self.start_time
        self.success = success
        return self

    def __str__(self):
        """
        Print the report card in a nice format
        """
        return """
== REPORT CARD ==
Success: {0.success}

Start time: {0.start_time}
End time: {0.end_time}
Total time: {0.total_time}

Total actions: {0.total_actions}
Total actions costs: {0.total_action_costs}
Failed actions: {0.failed_actions}
Total perception requests: {0.total_perception_requests}
""".format(self)
