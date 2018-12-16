
class KeyboardDataHandler():

    def __init__(self):
        self._command = None
        self._action = None
        self._action_opt = None

    def process_data(self, data):
        self._command = data.command
        self._action = data.action
        self._action_opt = data.action_opt
        print("[INFO] Class: {} ... Received message from keyboard-node: {}\n".format(self.__class__.__name__, data))

    # command
    @property
    def command(self):
        return self._command

    @command.setter
    def command(self, value):
        self._command = value

    #action
    @property
    def action(self):
        return self._action

    @action.setter
    def action(self, value):
        self._action = value

    #action_opt
    @property
    def action_opt(self):
        return self._action_opt

    @action_opt.setter
    def action_opt(self, value):
        self._action_opt = value
