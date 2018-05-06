SUPPORTS_LAPKT = False
try:
    from pddlsim.external.liblapkt import Planner
    SUPPORTS_LAPKT = True
except:
    pass


class ValidActions():

    def __init__(self, parser, perception, problem_generator, goal_tracking):
        problem = parser.problem_path

        self.provider = None
        if SUPPORTS_LAPKT:
            if goal_tracking.has_multiple_goals():
                problem = problem_generator.generate_problem(
                    goal_tracking.uncompleted_goals[0])
            self.provider = TrackedSuccessorValidActions(
                parser.domain_path, problem)
        else:
            self.provider = PythonValidActions(parser, perception)

    def get(self):
        return self.provider.get()

    def on_action(self, action_sig):
        self.provider.on_action(action_sig)


class TrackedSuccessorValidActions():

    """
    Use the TrackedSuccessor to query for valid actions at the current state
    This successor is tracked because LAPKT needs to keep track of the state
    """

    def __init__(self, domain_path, problem_path):
        self.task = Planner()
        self.task.load(domain_path, problem_path)
        self.task.setup()
        self.sig_to_index = dict()
        for i in range(0, self.task.num_actions()):
            self.sig_to_index[self.task.get_action_signature(i)] = i

    def get(self):
        return map(str.lower, self.task.next_actions_from_current())

    def on_action(self, action_signature):
        self.task.proceed_with_action(
            self.sig_to_index[action_signature.upper()])


class PythonValidActions():

    """
    Python implemention for valid actions
    This is significantly less efficient than the TrackedSuccessor version
    """

    def __init__(self, parser, perception):
        self.parser = parser
        self.perception = perception

    def get(self):
        current_state = self.perception.get_state()
        possible_actions = []
        for (name, action) in self.parser.actions.items():
            for candidate in self.get_valid_candidates_for_action(current_state, action):
                possible_actions.append(action.action_string(candidate))
        return possible_actions

    def join_candidates(self, previous_candidates, new_candidates, p_indexes, n_indexes):
        shared_indexes = p_indexes.intersection(n_indexes)
        if previous_candidates is None:
            return new_candidates
        result = []
        for c1 in previous_candidates:
            for c2 in new_candidates:
                if all([c1[idx] == c2[idx] for idx in shared_indexes]):
                    merged = c1[:]
                    for idx in n_indexes:
                        merged[idx] = c2[idx]
                    result.append(merged)
        return result

    def indexed_candidate_to_dict(self, candidate, index_to_name):
        return {name[0]: candidate[idx] for idx, name in index_to_name.items()}

    def on_action(self, action_sig):
        pass

    def get_valid_candidates_for_action(self, state, action):
        '''
        Get all the valid parameters for a given action for the current state of the simulation
        '''
        objects = dict()
        signatures_to_match = {
            name: (idx, t) for idx, (name, t) in enumerate(action.signature)}
        index_to_name = {idx: name for idx,
                         name in enumerate(action.signature)}
        candidate_length = len(signatures_to_match)
        found = set()
        candidates = None
        # copy all preconditions
        for precondition in sorted(action.precondition, key=lambda x: len(state[x.name])):
            thruths = state[precondition.name]
            if len(thruths) == 0:
                return []
            # map from predicate index to candidate index
            dtypes = [(name, 'object') for name in precondition.signature]
            reverse_map = {idx: signatures_to_match[pred][0] for idx, pred in enumerate(
                precondition.signature)}
            indexes = reverse_map.values()
            overlap = len(found.intersection(indexes)) > 0
            precondition_candidates = []
            for entry in thruths:
                candidate = [None] * candidate_length
                for idx, param in enumerate(entry):
                    candidate[reverse_map[idx]] = param
                precondition_candidates.append(candidate)

            candidates = self.join_candidates(
                candidates, precondition_candidates, found, indexes)
            # print( candidates)
            found = found.union(indexes)

        return [self.indexed_candidate_to_dict(c, index_to_name) for c in candidates]
