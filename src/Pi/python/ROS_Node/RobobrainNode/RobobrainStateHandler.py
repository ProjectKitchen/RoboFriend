class RobobrainStateHandler():

    def __init__(self, actual_state):
        self._state = actual_state

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = value
