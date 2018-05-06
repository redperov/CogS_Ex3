from executor import Executor
import resource
import pickle


class ExecutorProfiler(Executor):

    """The executor profile uses the decorator pattern to log the rusage for a given executive"""

    def __init__(self, target):
        super(ExecutorProfiler, self).__init__()
        self.target = target

    def log_rusage(self, key):
        self.rusage_log[key] = resource.getrusage(resource.RUSAGE_SELF)

    def save_rusage_log(self, path):
        with open(path, mode="w") as f:
            pickle.dump(self.rusage_log, f)

    def initialize(self, services):
        self.rusage_log = dict()
        self.action_count = 0

        self.log_rusage('pre_init')
        self.target.initialize(services)
        self.log_rusage('post_init')
        return None

    def next_action(self):
        next_action = self.target.next_action()
        self.log_rusage('a' + str(self.action_count))
        self.action_count += 1
        return next_action
