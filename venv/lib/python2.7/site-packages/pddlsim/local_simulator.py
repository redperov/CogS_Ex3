from simulator import Simulator
from fd_parser import FDParser
from services.simulator_services import SimulatorServices
from services.pddl import PDDL
from services.perception import Perception


class LocalSimulator:

    def __init__(self, print_actions=True, planner=None):
        self.print_actions = print_actions
        self.planner = planner

    def run(self, domain_path, problem_path, executive):
        parser = FDParser(domain_path, problem_path)
        sim = Simulator(parser)
        mediator = SimulatorServices(parser, sim.perceive_state, self.planner)
        if self.print_actions:
            def printer(text):
                print text
            mediator.on_action_observers.append(printer)
        executive.initialize(mediator)
        self.previous_action = None

        def next_action():
            if self.previous_action:
                mediator.on_action(self.previous_action)
            self.previous_action = executive.next_action()
            return self.previous_action

        return sim.simulate(next_action)
