from pddl import PDDL
from goal_tracking import GoalTracking
from valid_actions import ValidActions
from problem_generator import ProblemGenerator
from perception import Perception
from pddlsim.fd_parser import FDParser
from pddlsim.planner import make_plan


class SimulatorServices():

    def __init__(self, parser, perception_func, planner=None):

        self.parser = parser
        self.planner = planner or make_plan
        self.perception = Perception(perception_func)
        self.pddl = self.parser
        self.goal_tracking = GoalTracking(self.parser, self.perception)
        self.problem_generator = ProblemGenerator(
            self.perception, self.parser, "tmp_problem_generation")

        self.valid_actions = ValidActions(
            self.pddl, self.perception, self.problem_generator, self.goal_tracking)

        self.on_action_observers = [self.goal_tracking.on_action,
                                    self.valid_actions.on_action, self.perception.on_action]

    @staticmethod
    def from_pddls(domain_path, problem_path, perception_func, planner=None):
        parser = FDParser(domain_path, problem_path)
        return SimulatorServices(parser, perception_func, planner)

    def on_action(self, action_sig):
        for func in self.on_action_observers:
            func(action_sig)
