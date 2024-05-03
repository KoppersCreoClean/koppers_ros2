
class State(object):

    def __init__(self):
        print('Processing current state:', str(self))

    def on_event(self, event):
        """
        Handle events that are delegated to this State.
        """
        pass

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__

class CreateStratagyState(State):
    pass

class PlanPreCleanState(State):
    pass

class ExecutePreCleanState(State):
    pass

class PlanStartCleanState(State):
    pass

class ExecuteStartCleanState(State):
    pass

class PlanCleanState(State):
    pass

class ExecuteCleanState(State):
    pass

class PlanStopCleanState(State):
    pass

class ExecuteStopCleanState(State):
    pass

class WaitState(State):
    pass
