from pddlsim.local_simulator import LocalSimulator
from pddlsim.executors.executor import Executor
from pddlsim.planner import local
import sys
from PlanParser import PlanParser
from pddlsim.parser_independent import Conjunction


class SimpleFootballExecutor(Executor):
    """
    A simple football executor.
    """

    def __init__(self, plan):
        super(SimpleFootballExecutor, self).__init__()

        # The plan.
        self.P = plan

        # Stack.
        self.S = []

        # The initial behavior.
        self.b = self.P.getNode("score_goal")

    def initialize(self, services):
        self.services = services

    def next_action(self):
        pass
        # self._first_block()

    def _first_block(self):
        """
        The first part of the BIS algorithm.
        :return:
        """
        self.S.append(self.b)

        while self._exists_n_in_H():
            A = self._get_all_sequences_in_H()
            C = self._get_all_valid_sequences_in_H(A)
            self.b = self._choose()
            self.S.append(self.b)

    def _choose(self):
        pass

    def _exists_n_in_H(self):
        sequential_children = self.b.sequentialFollowers
        for n in sequential_children:
            if n in self.P:
                return True
        return False

    def _get_all_sequences_in_H(self):
        sequential_children = self.b.sequentialFollowers
        children_in_H = []
        for n in sequential_children:
            if n in self.P:
                children_in_H.append(n)
        return children_in_H

    def _get_all_valid_sequences_in_H(self, A):
        valid_sequences = []
        current_state = self.services.perception.get_state()

        for a in A:
            preconditions = Conjunction(a.preConds)

            # TODO maybe get the current state inside the loop
            # TODO check if test_condition returns a boolean
            if self.services.pddl.test_condition(preconditions, current_state):
                valid_sequences.append(a)

        return valid_sequences



# def exists_n_in_H(b, H):
#     sequential_children = b.sequentialFollowers
#     for n in sequential_children:
#         if n in H:
#             return True
#     return False
#
# def get_all_sequentials_in_H(b, H):
#     sequential_children = b.sequentialFollowers
#     children_in_H = []
#
#     for n in sequential_children:
#         if n in H:
#             children_in_H.append(n)
#
#     return children_in_H
#
# def get_all_valid_sequentials_in_H(A):
#     valid_sequentials = []
#    # current_state = self.services.perception.get_state()
#
#     for a in A:
#         #temp_state = current_state
#         #self.services.pddl.apply_action_to_state(option, future_state)
#
#
#
# def BIS(P):
#
#     # Stack.
#     S = []
#
#     # Initial node.
#     # TODO should I get the initial behavior dynamically?
#     b = P.getNode("score_goal")
#     S.append(b)
#
#     while exists_n_in_H(b, plan):
#         A = get_all_sequentials_in_H(b, plan)
#         C = get_all_valid_sequentials_in_H(A)
#
#
#


if __name__ == "__main__":

    # Flag that indicates which plan to use.
    flag = sys.argv[1]

    # Number of balls.
    N = sys.argv[2]

    # Problem file.
    problem_file = sys.argv[3]



    # Check which plan to use.
    if flag == "-s":
        plan_path = "simple_football_plan.xml"
        domain_path = "simple_football_domain.pddl"
        problem_path = "simple_football_problem.pddl"

        # Get the plan.
        plan = PlanParser(plan_path).getPlan()

        executor = SimpleFootballExecutor(plan)
    elif flag == "-e":
        plan_path = "extended_football_plan.xml"
        domain_path = "extended_football_domain.pddl"
        problem_path = "simple_football_problem.pddl"
        executor = None  # ExtendedFootballExecutor()
    else:
        plan_path = ""
        domain_path = ""
        problem_path = ""
        executor = None

    print LocalSimulator(local).run(domain_path, problem_path, executor)

