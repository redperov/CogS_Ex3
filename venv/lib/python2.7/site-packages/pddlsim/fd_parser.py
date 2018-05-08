
import external.fd.pddl as pddl
from six import iteritems, print_
import abc
from parser_independent import *


class FDParser(PDDL):

    def __init__(self, domain_path, problem_path):
        self.task = pddl.pddl_file.open(problem_path, domain_path)
        objects = {obj.name: obj.type for obj in self.task.objects}
        actions = {action.name: self.convert_action(action)
                   for action in self.task.actions}
        goals = [self.convert_condition(subgoal)
                 for subgoal in self.task.goal]

        super(FDParser, self).__init__(
            domain_path, problem_path, self.task.domain_name, self.task.task_name, objects, actions, goals, self.build_first_state())

    def build_first_state(self):
        initial_state = self.task.init
        current_state = dict()
        for predicate in self.task.predicates:
            current_state[predicate.name] = set()
        for atom in initial_state:
            current_state[atom.key[0]].add(atom.key[1])
        return current_state

    @staticmethod
    def convert_condition(condition):
        if isinstance(condition, pddl.Literal):
            literal = Literal(condition.predicate, condition.args)
            return literal if not condition.negated else Not(literal)

        sub_conditions = [FDParser.convert_condition(sub_condition)
                          for sub_condition in condition.parts]
        if isinstance(condition, pddl.Conjunction):
            return Conjunction(sub_conditions)
        if isinstance(condition, pddl.Disjunction):
            return Disjunction(sub_conditions)

    @staticmethod
    def convert_action(action):
        name = action.name
        signature = [(obj.name, obj.type) for obj in action.parameters]
        addlist = []
        dellist = []
        for effect in action.effects:
            if effect.literal.negated:
                dellist.append(effect.literal.key)
            else:
                addlist.append(effect.literal.key)
        precondition = [Predicate(pred.predicate, pred.args, pred.negated)
                        for pred in action.precondition.parts]
        return Action(action.name, signature, addlist, dellist, precondition)
