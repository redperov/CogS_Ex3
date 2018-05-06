from executor import Executor


class PlanDispatcher(Executor):

    """docstring for PlanDispatcher."""

    def __init__(self):
        super(PlanDispatcher, self).__init__()
        self.steps = []

    def initialize(self, services):
        self.steps = services.planner(
            services.pddl.domain_path, services.pddl.problem_path)

    def next_action(self):
        if len(self.steps) > 0:
            return self.steps.pop(0).lower()
        return None
