import abc

REACH_GOAL = 'reach-goal'


class PDDL(object):

    def __init__(self, domain_path, problem_path, domain_name, problem_name, objects, actions, goals, initial_state):
        """
        :param domain_path: - path of the domain file used
        :param problem_path: - path of the problem file used
        :param domain_name:
        :param problem_name:
        :param objects: objects in the problem, in the format of a dictionary from name to type
        :param actions: action of type Action
        :param goals: a list of Condition
        :param initial_state: the initial state of the problem
        """
        self.domain_path = domain_path
        self.domain_name = domain_name
        self.problem_path = problem_path
        self.problem_name = problem_name
        self.objects = objects
        self.actions = actions
        self.goals = goals
        self.initial_state = initial_state

    def build_first_state(self):
        return self.copy_state(self.initial_state)

    def get_object(self, name):
        """ Get a object tuple for a name """
        if name in self.objects:
            return (name, self.objects[name])

    def test_condition(self, condition, mapping):
        return condition.test(mapping)

    def pd_to_strips_string(self, condition):
        return condition.accept(StripsStringVisitor())

    def predicates_from_state(self, state):
        return [("(%s %s)" % (predicate_name, " ".join(map(str, pred)))) for predicate_name, predicate_set in state.iteritems() for pred in predicate_set if predicate_name != '=']

    def generate_problem(self, path, state, new_goal):
        predicates = self.predicates_from_state(state)
        goal = self.pd_to_strips_string(new_goal)
        # goal = self.tuples_to_string(new_goal)
        with open(path, 'w') as f:
            f.write('''
    (define (problem ''' + self.problem_name + ''')
    (:domain  ''' + self.domain_name + ''')
    (:objects
        ''')
            for t in self.objects.keys():
                f.write('\n\t' + t)
            f.write(''')
(:init
''')
            f.write('\t' + '\n\t'.join(predicates))
            f.write('''
            )
    (:goal
        ''' + goal + '''
        )
    )
    ''')

    @staticmethod
    def parse_action(action):
        action_sig = action.strip('()').lower()
        parts = action_sig.split(' ')
        action_name = parts[0]
        param_names = parts[1:]
        return action_name, param_names

    def apply_action_to_state(self, action_sig, state, check_preconditions=True):
        action_name, param_names = self.parse_action(action_sig)
        if action_name.lower() == REACH_GOAL:
            return state
        action = self.actions[action_name]
        params = map(self.get_object, param_names)

        param_mapping = action.get_param_mapping(params)

        if check_preconditions:
            for precondition in action.precondition:
                if not precondition.test(param_mapping, state):
                    raise PreconditionFalseError()

        for (predicate_name, entry) in action.to_delete(param_mapping):
            predicate_set = state[predicate_name]
            if entry in predicate_set:
                predicate_set.remove(entry)

        for (predicate_name, entry) in action.to_add(param_mapping):
            state[predicate_name].add(entry)

    def copy_state(self, state):
        return {name: set(entries) for name, entries in state.items()}


class Action(object):

    def __init__(self, name, signature, addlist, dellist, precondition):
        self.name = name
        self.signature = signature
        self.addlist = addlist
        self.dellist = dellist
        self.precondition = precondition

    def action_string(self, dictionary):
        params = " ".join([dictionary[var[0]] for var in self.signature])
        return "(" + self.name + " " + params + ")"

    @staticmethod
    def get_entry(param_mapping, predicate):
        names = [x for x in predicate]
        entry = tuple([param_mapping[name][0] for name in names])
        return entry

    def entries_from_list(self, preds, param_mapping):
        return [(pred[0], self.get_entry(param_mapping, pred[1])) for pred in preds]

    def to_delete(self, param_mapping):
        return self.entries_from_list(self.dellist, param_mapping)

    def to_add(self, param_mapping):
        return self.entries_from_list(self.addlist, param_mapping)

    def get_param_mapping(self, params):
        param_mapping = dict()
        for (name, param_type), obj in zip(self.signature, params):
            param_mapping[name] = obj
        return param_mapping


class Predicate(object):

    def __init__(self, name, signature, negated=False):
        self.name = name
        self.signature = signature
        self.negated = negated

    def ground(self, dictionary):
        return tuple([dictionary[x][0] for x in self.signature])

    def test(self, param_mapping, state):
        result = self.ground(param_mapping) in state[self.name]
        return result if not self.negated else not result


class ConditionVisitor():
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def visit_literal(self, literal):
        pass

    @abc.abstractmethod
    def visit_not(self, negation):
        pass

    @abc.abstractmethod
    def visit_conjunction(self, conjunction):
        pass

    @abc.abstractmethod
    def visit_disjunction(self, disjunction):
        pass


class Condition():
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def accept(self, visitor):
        pass

    @abc.abstractmethod
    def test(self, mapping):
        pass


class Literal(Condition):

    def __init__(self, predicate, args):
        self.predicate = predicate
        self.args = tuple(args)

    def accept(self, visitor):
        return visitor.visit_literal(self)

    def test(self, mapping):
        return self.args in mapping[self.predicate]


class Not(Condition):

    def __init__(self, content):
        self.content = content

    def accept(self, visitor):
        return visitor.visit_not(self)

    def test(self, mapping):
        return not self.content.test(mapping)


class JunctorCondition(Condition):

    def __init__(self, parts):
        self.parts = parts


class Conjunction(JunctorCondition):

    def accept(self, visitor):
        return visitor.visit_conjunction(self)

    def test(self, mapping):
        return all([part.test(mapping) for part in self.parts])


class Disjunction(JunctorCondition):

    def accept(self, visitor):
        return visitor.visit_disjunction(self)

    def test(self, mapping):
        return any([part.test(mapping) for part in self.parts])


class StripsStringVisitor(ConditionVisitor):

    def visit_literal(self, condition):
        return "({} {})".format(condition.predicate, ' '.join(condition.args))

    def visit_not(self, condition):
        return "(not {})".format(condition.content.accept(self))

    def join_parts(self, condition):
        return ' '.join([part.accept(self) for part in condition.parts])

    def visit_conjunction(self, condition):
        return "(and {})".format(self.join_parts(condition))

    def visit_disjunction(self, condition):
        return "(or {})".format(self.join_parts(condition))


class PreconditionFalseError(Exception):
    pass
