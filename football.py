from pddlsim.local_simulator import LocalSimulator
from pddlsim.executors.executor import Executor
from pddlsim.planner import local
import sys
from PlanParser import PlanParser
from pddlsim.parser_independent import Conjunction, Predicate, Literal
from grid import Grid


class SimpleFootballExecutor(Executor):
    """
    A simple football executor.
    """

    def __init__(self, plan, num_of_balls):
        super(SimpleFootballExecutor, self).__init__()

        # Number of balls on the field.
        self.num_of_balls = num_of_balls

        # The plan.
        self.P = plan

        # Create the football grid.
        self.grid = Grid()
        self.goal_position = self.grid.get_tile("goal-tile")

        # Stack.
        self.S = []

        # The initial behavior.
        self.b = self.P.getNode("score_goal")

    def initialize(self, services):
        self.services = services

    def next_action(self):

        if self.num_of_balls == 0:
            return None

        self._first_block()
        self.b = self.S.pop()

        return self.b

    def _first_block(self):
        """
        The first part of the BIS algorithm.
        Lines 1-8.
        :return: None
        """
        self.S.append(self.b)

        while self._exists_n_in_H():
            A = self._get_all_sequences_in_H()
            C = self._get_all_valid_sequences_in_H(A)
            self.b = self._choose(C)
            self.S.append(self.b)

    # def _update(self):
    #     pass
    #
    # def _revise(self):
    #     pass
    #
    # def _check_termconditions(self):
    #     pass
    #
    # def _third_block(self):
    #     """
    #     Lines 11-15.
    #     :return: None
    #     """
    #     E = []
    #
    #     while len(E) == 0:
    #         K = self._update()
    #         W = self._revise()
    #         E = self._check_termconditions()

    def _get_agent_position(self, state):
        """
        Returns the agents position in the given state.
        :param state: state
        :return: Tile name
        """
        set_state = state["at-robby"]
        return set_state.pop()[0]

    def _get_balls_positions(self, state):
        """
        Returns all the balls positions on the field.
        :param state: state
        :return: list of Tile
        """
        balls = state['at-ball']
        balls_positions = {}

        for ball in balls:
            tile = self.grid.get_tile(ball[1])
            balls_positions[tile.get_name()] = tile

        return balls_positions

    def can_kick(self, valid_actions):
        """
        Checks if the agent can perform a kick.
        :param valid_actions: list of valid actions
        :return: boolean
        """

        for action in valid_actions:
            if action[0] == "kick":
                return True

        return False

    def _choose_best_kick(self, balls_positions, valid_actions):
        """
        Chooses the best kick action the agent can take.
        :param balls_positions: list of balls positions
        :param valid_actions: list of valid actions
        :return: best kick action
        """
        shortest_distance = float("inf")
        best_kick = None

        # Get all the valid kicks.
        for action in valid_actions:

            # Check if the action is "kick".
            if action[0] == "kick":
                name_of_ball = action[1]
                ball = balls_positions[name_of_ball]

                # Calculate the distance of the resulted kick to the goal tile.
                distance = self.grid.get_distance(ball, self.goal_position)

                # Check if the position of the ball after the kick is closer to the goal.
                if distance < shortest_distance:
                    shortest_distance = distance
                    best_kick = action

        return best_kick

    def _find_nearest_ball(self, agent_position, balls_positions):
        """
        Finds the nearest ball to the agent.
        :param agent_position: agent's position
        :param balls_positions: list of balls positions of object Tile
        :return: Tile
        """
        shortest_distance = float("inf")
        nearest_ball = None

        for ball in balls_positions:

            # Calculate the distance between the agent and the ball.
            distance = self.grid.get_distance(agent_position, ball)

            # Check if the current ball is the closest of all the previous ones.
            if distance < shortest_distance:
                shortest_distance = distance
                nearest_ball = ball

        return nearest_ball

    def _choose_best_move(self, agent_position, valid_actions, balls_positions):
        """
        Finds the move that gets the agent the closest to the nearest ball.
        :param agent_position: agent's position
        :param valid_actions: list of valid actions
        :param balls_positions: list of balls positions
        :return: best move action
        """
        shortest_distance = float("inf")
        best_move = None

        for action in valid_actions:

            # Check if the action is "kick".
            if action[0] == "move":
                # TODO find the move that gets the agent the closest to the nearest ball
                ball = self._find_nearest_ball(agent_position, balls_positions)
                next_tile = action[2]

                # Calculate the distance between the agent and the ball.
                distance = self.grid.get_distance(next_tile, ball)

                # Check if the current ball is the closest of all the previous ones.
                if distance < shortest_distance:
                    shortest_distance = distance
                    best_move = action

        return best_move

    def _choose(self, valid_actions):
        """
        The simple football domain choose action.
        :param valid_behaviors: list of valid behaviors as tuples
        :return: action
        """
        # Check if there are valid actions to choose from.
        if len(valid_actions) == 0:
            return None

        # Get the agent's position.
        current_state = self.services.perception.get_state()
        agent_position = self._get_agent_position(current_state)

        # Get the balls' positions.
        balls_positions = self._get_balls_positions(current_state)

        # Check if the agent can kick, else move
        if self.can_kick(valid_actions):
            action = self._choose_best_kick(balls_positions, valid_actions)
            # TODO choose which ball to kick(if next to more than one), then choose where to kick
        else:
            # TODO find where is the nearest ball, move towards it
            action = self._choose_best_move(agent_position, valid_actions, balls_positions)

        # Convert action tuple to string.
        str_action = "({0})".format(" ".join(action))

        return str_action

    def _exists_n_in_H(self):

        # Try to get the hierarchical children from the current behavior.
        try:
            hierarchical_children = self.b.hierarchicalChildren
        except AttributeError:
            return False

        # Check if at least one of the children exists in the plan.
        for n in hierarchical_children:
            if n in self.P.nodes:
                return True
        return False

    def _get_all_sequences_in_H(self):
        hierarchical_children = self.b.hierarchicalChildren
        children_in_H = []
        for n in hierarchical_children:
            if n in self.P.nodes:
                children_in_H.append(n)
        return children_in_H

    # def _precondition_to_predicate(self, preconditions):
    #     predicates = []
    #
    #     for precond in preconditions:
    #         pred_stuff = precond.split()
    #         signature = tuple(pred_stuff[1:])
    #         negated = True if preconditions[precond][0] == "True" else False
    #         predicate = Predicate(pred_stuff[0], signature, negated)
    #         predicates.append(predicate)
    #
    #     return predicates

    def _convert_str_actions_to_tuples(self, str_actions):
        tupled_actions = []

        for action in str_actions:
            splitted_action = action[1:-1].split()
            tupled_actions.append(tuple(splitted_action))

        return tupled_actions
    #
    # def _get_valid_args(self, action_name):
    #
    #     valid_actions = self._convert_str_actions_to_tuples(self.services.valid_actions.get())
    #     valid_args = []
    #
    #     for action in valid_actions:
    #         if action[0] == action_name:
    #             valid_args.append(action[1:])
    #
    #     return valid_args

    def _exists_behavior(self, action, hierarchical_behaviors):
        """
        Check if the action exists in the hierarchical behaviors list.
        :param action: action
        :param hierarchical_behaviors: list of behaviors
        :return: boolean
        """
        for behavior in hierarchical_behaviors:
            if behavior.action == action[0]:
                return True

        return False

    def _get_all_valid_sequences_in_H(self, hierarchical_behaviors):
        # TODO should I use test_condition instead?
        all_valid_actions = self._convert_str_actions_to_tuples(self.services.valid_actions.get())
        valid_actions_for_behaviors = []

        for action in all_valid_actions:
            if self._exists_behavior(action, hierarchical_behaviors):
                valid_actions_for_behaviors.append(action)

        return valid_actions_for_behaviors
        # valid_sequences = []
        # current_state = self.services.perception.get_state()
        #
        # for a in A:
        #
        #     # Creating the conjunction that will be passed to the test_condition method.
        #     predicates = self._precondition_to_predicate(a.preConds)
        #
        #     # If there are no preconditions, no need to keep checking.
        #     if len(predicates) == 0:
        #         continue
        #
        #     valid_args = self._get_valid_args(a.action)
        #
        #     for args in valid_args:
        #         # TODO the predicate.signature is wrong, change it
        #         literal = Literal(predicate.name, args)
        #         conjunction = Conjunction([literal])
        #
        #         # TODO maybe get the current state inside the loop
        #         # TODO check if test_condition returns a boolean
        #         t = self.services.pddl.test_condition(conjunction, current_state)
        #         if self.services.pddl.test_condition(conjunction, current_state):
        #             valid_sequences.append(a)
        #
        # return valid_sequences


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

        executor = SimpleFootballExecutor(plan, N)
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
