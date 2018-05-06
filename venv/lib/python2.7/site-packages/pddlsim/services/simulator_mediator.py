from pddl import PDDL
from goal_tracking import GoalTracking
from valid_actions import TrackedSuccessorValidActions, PythonValidActions
from problem_generator import ProblemGenerator
from action_simulator import ActionSimulator
from perception import Perception
from pddlsim.fd_parser import FDParser


class SimulatorMediator():
    def __init__(self, parser, perception):

        self.parser = parser
        self.perception = perception
        self.pddl = PDDL(parser.domain_path, parser.problem_path)
        self.goal_tracking = GoalTracking(self.parser, self.perception)
        self.problem_generator = ProblemGenerator(
            self.perception, self.parser, "tmp_problem_generation")
        self.action_simulator = ActionSimulator(
            self.parser, self.perception)

        self.valid_actions = TrackedSuccessorValidActions(
            self.pddl, self.problem_generator, self.goal_tracking)

        self.on_action_observers = [self.goal_tracking.on_action,
                                    self.valid_actions.on_action, self.perception.on_action]

    def on_action(self, sim, action_sig):
        for func in self.on_action_observers:
            func(action_sig)
