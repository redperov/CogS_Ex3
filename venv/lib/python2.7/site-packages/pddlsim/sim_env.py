from simulator import Simulator
from fd_parser import FDParser
from services.simulator_mediator import SimulatorMediator
from services.pddl import PDDL
from services.perception import Perception


class SimEnv:
    def __init__(self, print_actions=True):
        self.print_actions = print_actions

    def run(self, domain_path, problem_path, executive):
        parser = FDParser(domain_path, problem_path)
        sim = Simulator(parser)
        perception = Perception(sim.perceive_state)
        mediator = SimulatorMediator(parser, perception)
        if self.print_actions:
            def printer(text):
                print text
            mediator.on_action_observers.append(printer)
        executive.initilize(mediator)
        self.previous_action = None

        def next_action():
            if self.previous_action:
                mediator.on_action(sim, self.previous_action)
            self.previous_action = executive.next_action()
            return self.previous_action

        return sim.simulate(next_action)
