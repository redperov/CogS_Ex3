class ProblemGenerator():

    """
    Used to generate a new problem pddl file
    Can be used to specify a different goal and/or a different starting position
    Note that this uses perception to get the current state if no state is specified
    """

    def __init__(self, perception, parser, path):
        self.perception = perception
        self.parser = parser
        self.path = path

    def generate_problem(self, goal, state=None):
        if state is None:
            state = self.perception.get_state()
        self.parser.generate_problem(self.path, state, goal)
        return self.path
