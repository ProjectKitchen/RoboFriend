
class RobobrainKeyboardDataHandler():

    def __init__(self):
        self.__quit = None
        self.__up_down = None
        self.__pressed_key = None

    def process_data(self, data):
        self.__quit = data.quit
        self.__up_down = data.up_down
        self.__pressed_key = data.pressed_key
        print("[INFO] Class: {} ... Received message from keyboard-node: {}\n".format(self.__class__.__name__, data))

    # command
    @property
    def quit(self):
        return self.__quit

    @quit.setter
    def quit(self, value):
        self.__quit = value

    # up_down
    @property
    def up_down(self):
        return self.__up_down

    @up_down.setter
    def up_down(self, value):
        self._action = value

    #action_opt
    @property
    def pressed_key(self):
        return self.__pressed_key

    @pressed_key.setter
    def pressed_key(self, value):
        self.__pressed_key = value
