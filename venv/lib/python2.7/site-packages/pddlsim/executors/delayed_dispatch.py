import random
import copy

from avoid_return_random import AvoidReturn
from pddlsim.simulator import Simulator
from pddlsim.planner import make_plan


class DelayedDispatch(AvoidReturn):
    def __init__(self):
        super(DelayedDispatch, self).__init__()
        self.previous_state = None

    def initilize(self,services):
        super(DelayedDispatch, self).initilize(services)        
        self.plan = None

    def next_step_in_plan(self):
        if self.plan:
                return self.plan.pop(0).lower()
        return None

    def next_action(self):
        if not self.plan is None:
            return self.next_step_in_plan()
        return super(DelayedDispatch,self).next_action()

    def pick_from_many(self, options):
        options = self.remove_return_actions(options)
        # if there is only one options after reducing actions that take you back
        if len(options) == 1: return options[0]
        # create pddl_problem for current state
        problem_path = self.services.problem_generator.generate_problem(self.services.goal_tracking.uncompleted_goals[0], self.services.perception.get_state())
        self.plan = make_plan(self.services.pddl.domain_path,problem_path)
        return self.next_step_in_plan()
