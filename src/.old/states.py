from abc import ABCMeta, abstractmethod

from .singleton import SingletonABCMeta


class StateABC(metaclass=ABCMeta):
    @abstractmethod
    def enter(self, entity):
        return

    @abstractmethod
    def execute(self, entity):
        return

    @abstractmethod
    def exit(self, entity):
        return

    @abstractmethod
    def on_message(self, entity, telegram):
        return

#pylint: disable=W0223
class SingletonStateABC(StateABC, metaclass=SingletonABCMeta):
    pass
#pylint: enable=W0223

class StateMachine(object):
    def __init__(self, owner):
        self._owner = owner
        self._current_state = None
        self._previous_state = None
        self._global_state = None

    def get_current_state(self):
        return self._current_state

    def set_current_state(self, state):
        self._current_state = state

    def get_previous_state(self):
        return self._previous_state

    def set_previous_state(self, state):
        self._previous_state = state

    def get_global_state(self):
        return self._global_state

    def set_global_state(self, state):
        self._global_state = state

    current_state = property(get_current_state, set_current_state)
    previous_state = property(get_previous_state, set_previous_state)
    global_state = property(get_global_state, set_global_state)

    def update(self):
        if self.global_state:
            self.global_state.execute(self._owner)
        if self.current_state:
            self.current_state.execute(self._owner)

    def change_state(self, new_state):
        if not new_state:
            raise TypeError("Trying to change to a NULL (None) state")

        owner = self._owner
        self.previous_state = self.current_state
        self.current_state.exit(owner)
        self.current_state = new_state
        self.current_state.enter(owner)

    def revert_to_previous_state(self):
        self.change_state(self.previous_state)

    def handle_message(self, message):
        if (self.current_state and
                self.current_state.on_message(self._owner, message)):
            return True
        if (self.global_state and
                self.global_state.on_message(self._owner, message)):
            return True
        return False

    def is_in_state(self, state_cls):
        return isinstance(self.current_state, state_cls)
