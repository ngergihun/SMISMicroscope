"""A module that contains the Motor class, which is the base class for all motors."""
from .cmd import switch_
from .device import DeviceWithAddress
from enum import Enum

class switch_states(Enum):
    open = 1
    closed = 0
    
class BaseSwitch(DeviceWithAddress):

    last_state = None
    state = None
    
    def __init__(self, controller, address="0", debug=False):
        super().__init__(controller, address, debug)
    
    def get_state(self):
        raise NotImplementedError

class SwitchSensor(BaseSwitch):
    def __init__(self, controller, address="0", openstate=1, debug=False):
        super().__init__(controller, address=address, debug=debug)

        self.openstate = openstate

    def get_state(self):
        """Returns the position of the servo in PWM value"""
        code, addr, data = self.dict_command(switch_,req="get_state")
        
        if data[0] == self.openstate and data[0] in [0,1]:
            new_state = switch_states.open
        elif data[0] != self.openstate and data[0] in [0,1]:
            new_state = switch_states.closed
        else:
            new_state = None

        if code == "SW":
            self.last_state = self.state
            self.state = new_state
            return new_state
        
        return None