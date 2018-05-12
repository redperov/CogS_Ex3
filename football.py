from pddlsim.local_simulator import LocalSimulator
from pddlsim.executors.executor import Executor
from pddlsim.planner import local
import sys
from PlanParser import PlanParser
from pddlsim.parser_independent import Conjunction, Predicate, Literal
from grid import Grid


class FootballExecutor(Executor):
    """
    A football executor.
    """

    def __init__(self, plan, num_of_balls, flag):
        super(FootballExecutor, self).__init__()

        # The plan.
        self.P = plan

        # Number of balls on the field.
        self.num_of_balls_left = num_of_balls

        # Indicating the type of the problem.
        self.flag = flag

        # Create the football grid.
        self.grid = Grid()
        self.goal_position = self.grid.get_tile("goal_tile").get_name()

        # Flag indicating whether the action performed is a kick.
        self.performing_kick_to_goal = False

        self.right_leg_position = None
        self.left_leg_position = None
        self.last_leg_moved = None
        self.lifted_leg = None


        # Stack.
        self.S = []

        # The initial behavior.
        self.b = self.P.nodes[0]
        self.S.append(self.b)

    def initialize(self, services):
        self.services = services

    def next_action(self):

        # Check if the previous action performed is a kick.
        if self.performing_kick_to_goal:
            self.num_of_balls_left -= 1
            self.performing_kick_to_goal = False

        # Check if there are balls left to score.
        if self.num_of_balls_left == 0:
            return None

        self._first_block()
        action = self.S.pop()
        self.b = self.S[-1]

        return action

    def _first_block(self):
        """
        The first part of the BIS algorithm.
        Lines 1-8.
        :return: None
        """

        while self._exists_n_in_H():
            A = self._get_all_sequences_in_H()
            C = self._get_all_valid_sequences_in_H(A)
            self.b = self._choose(C)
            self.S.append(self.b)

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
            tile_name = tile.get_name()

            # Check if the ball wasn't already scored.
            if tile_name != "goal_tile":
                balls_positions[tile_name] = tile

        return balls_positions

    def can_kick(self, valid_actions):
        """
        Checks if the agent can perform a kick.
        :param valid_actions: list of valid actions
        :return: boolean
        """

        for action in valid_actions:
            if action[0].startswith("kick"):
                return True

        return False

    def _choose_best_kick(self, valid_actions):
        """
        Chooses the best kick action the agent can take.
        :param valid_actions: list of valid actions
        :return: best kick action
        """
        shortest_distance = float("inf")
        best_kick = None

        # Get all the valid kicks.
        for action in valid_actions:

            # Check if the action is "kick".
            if action[0] == "kick":
                next_ball_position = action[3]

                # Calculate the distance of the resulted kick to the goal tile.
                distance = self.grid.get_distance(next_ball_position, self.goal_position)

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

            # Check if the action is "move".
            if action[0] == "move":

                # Find the nearest ball.
                ball = self._find_nearest_ball(agent_position, balls_positions)
                next_tile = action[2]

                # Calculate the distance between the agent and the ball.
                distance = self.grid.get_distance(next_tile, ball)

                # Check if the current ball is the closest of all the previous ones.
                if distance < shortest_distance:
                    shortest_distance = distance
                    best_move = action

        return best_move

    def _simple_choose(self, valid_actions):
        """
        The simple football domain choose action.
        :param valid_actions: list of valid actions as tuples
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

            # Choose which ball to kick(if next to more than one), then choose where to kick.
            action = self._choose_best_kick(valid_actions)

            # Check if kicking to the goal tile.
            if action[3] == "goal_tile":
                self.performing_kick_to_goal = True
        else:

            # Find where is the nearest ball and move towards it.
            action = self._choose_best_move(agent_position, valid_actions, balls_positions)

        # Convert action tuple to string.
        str_action = "({0})".format(" ".join(action))

        return str_action

    def _choose_leg_to_move_with(self):
        """
        Chooses a leg with which the agent can move.
        :return: leg
        """
        # TODO maybe make a smarter decision if can use both legs
        if self.last_leg_moved == "left":
            return "right"
        else:
            return "left"

    def _choose_best_move_with_leg(self, leg_to_move_with, agent_position, valid_actions, balls_positions):
        """
        Finds the move with the given leg that gets the agent the closest to the nearest ball.
        :param leg_to_move_with: leg with which the agent will move
        :param agent_position: agent's position
        :param valid_actions: list of valid actions
        :param balls_positions: list of balls positions
        :return: best move action
        """
        shortest_distance = float("inf")
        best_move = None
        move_action_name = "move-{0}-leg".format(leg_to_move_with)

        for action in valid_actions:

            # Check if the action is "move".
            if action[0] == move_action_name:

                # Find the nearest ball.
                ball = self._find_nearest_ball(agent_position, balls_positions)
                next_tile = action[2]

                # Calculate the distance between the agent and the ball.
                distance = self.grid.get_distance(next_tile, ball)

                # Check if the current ball is the closest of all the previous ones.
                if distance < shortest_distance:
                    shortest_distance = distance
                    best_move = action

        return best_move

    def _choose_leg_to_lift(self):
        """
        Chooses a leg to lift for a kick.
        :return: leg
        """
        # TODO maybe make a smarter decision if can use both legs
        if self.last_leg_moved == "left":
            return "right"
        else:
            return "left"

    def _choose_lift_action(self, leg_to_lift, valid_actions):
        lift_action = None

        for action in valid_actions:


    def _extended_choose(self, valid_actions):
        """
        The extended football domain choose action.
        :param valid_actions: list of valid actions as tuples
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

            # TODO first, if the leg is not raised, raise it
            # TODO if the leg is raised, kick with it

            if self.lifted_leg is None:
                leg_to_lift = self._choose_leg_to_lift()
                action = self._choose_lift_action(leg_to_lift, valid_actions)

                self.lifted_leg = leg_to_lift
            else:
                # Choose which ball to kick(if next to more than one), then choose where to kick.
                action = self._choose_best_kick(valid_actions)

            # Check if kicking to the goal tile.
            if action[3] == "goal_tile":
                self.performing_kick_to_goal = True
        else:

            # TODO first choose a leg with which it can move(remember which leg was used the last time)
            # TODO then choose the move with that leg that will get the agent the closest to the ball

            leg_to_move_with = self._choose_leg_to_move_with()
            action = self._choose_best_move_with_leg(leg_to_move_with, agent_position, valid_actions, balls_positions)

            self.last_leg_moved = leg_to_move_with

            # Find where is the nearest ball and move towards it.
            # action = self._choose_best_move(agent_position, valid_actions, balls_positions)

        # Convert action tuple to string.
        str_action = "({0})".format(" ".join(action))

        return str_action

    def _choose(self, valid_actions):
        """
        Decides which choose method to use.
        :param valid_actions: list of valid actions as tuples.
        :return: action
        """
        if self.flag == "-s":
            return self._simple_choose(valid_actions)
        elif self.flag == "-e":
            return self._extended_choose(valid_actions)
        else:
            return None

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

    def _convert_str_actions_to_tuples(self, str_actions):
        """
        Converts actions in string representation to tuples.
        :param str_actions: list of actions as strings
        :return: list of actions as tuples
        """
        tupled_actions = []

        for action in str_actions:
            splitted_action = action[1:-1].split()
            tupled_actions.append(tuple(splitted_action))

        return tupled_actions

    def _exists_behavior(self, action, hierarchical_behaviors):
        """
        Check if the action exists in the hierarchical behaviors list.
        :param action: action
        :param hierarchical_behaviors: list of behaviors
        :return: boolean
        """
        for behavior in hierarchical_behaviors:
            if action[0].startswith(behavior.action):
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


if __name__ == "__main__":

    # Flag that indicates which plan to use.
    flag = sys.argv[1]

    # Number of balls.
    N = int(sys.argv[2])

    # Problem file.
    plan_path = sys.argv[3]

    # Check which plan to use.
    if flag == "-s":
        domain_path = "simple_football_domain.pddl"
        problem_path = "simple_football_problem.pddl"
    elif flag == "-e":
        domain_path = "extended_football_domain.pddl"
        problem_path = "extended_football_problem.pddl"
    else:
        domain_path = ""
        problem_path = ""

    # Get the plan.
    plan = PlanParser(plan_path).getPlan()

    # Create the executor.
    executor = FootballExecutor(plan, N, flag)

    print LocalSimulator(local).run(domain_path, problem_path, executor)
