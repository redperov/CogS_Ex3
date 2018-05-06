from pddlsim.external.liblapkt import Planner 

# class for profiling purposes
class Planner_Wrapper(object):
    def __init__(self):
        self.task = Planner()
    
    def setup(self):
        self.task.setup()

    def create_state(self,encoded):
        return self.task.create_state(encoded)
    
    def next_actions(self,state):
        return self.task.next_actions(state)
    
    def encode(self, atoms, atom_table):
        return self.task.encode(atoms,atom_table)

    def next_actions_from_atoms(self,atoms,atom_table):
        return self.task.next_actions_from_atoms(atoms,atom_table)