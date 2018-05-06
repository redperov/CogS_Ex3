from pddlsim.external.liblapkt import Planner 


from planner_wrapper import Planner_Wrapper

class TrackedSuccessor(object):
    """
    this class keeps uses liblapkt inorder to keep track of the state of the problem
    this allows for swift calculation of the valid next actions
    """
    def __init__(self, domain_path, problem_path):
        self.task = Planner()        
        self.task.load(domain_path,problem_path)
        self.task.setup()
        self.sig_to_index = dict()
        for i in range( 0, self.task.num_actions() ) :
            self.sig_to_index[self.task.get_action_signature( i )] = i
    
    def next(self):
        return self.task.next_actions_from_current()

    def proceed(self, action_signature):
        self.task.proceed_with_action(self.sig_to_index[action_signature.upper()])
