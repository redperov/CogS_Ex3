class Perception():
    """
    Perception is wrapper around a real perception function to
     ensure that the real function is called only when the state has changed
    """

    def __init__(self, perception_func):
        self.perception_func = perception_func
        self.state = None
        self.dirty = True

    def get_state(self):
        if self.dirty:
            self.state = self.perception_func()
            self.dirty = False
        return {name: set(entries) for name, entries in self.state.items()}

    def on_action(self, action):
        self.dirty = True
