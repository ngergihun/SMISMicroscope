"""A module that contains the Motor class, which is the base class for all motors."""

from enum import Enum
from .cmd import shutter_
from .device import DeviceWithAddress
from .switch import SwitchSensor, switch_states
from enum import Enum

class servo_states(Enum):
    A = 0
    B = 1

class BaseServo(DeviceWithAddress):

    position = None
    last_position = None
    last_state = None
    state = None
    
    def __init__(self, controller, address="0", debug=False):
        super().__init__(controller, address, debug)

    def set_position(self):
        raise NotImplementedError
    
    def get_position(self):
        raise NotImplementedError
    
    def flip(self):
        raise NotImplementedError

class PwmServo(BaseServo):
    def __init__(self, controller, address="0", posA = 120, posB = 280, tol = 5, debug=False):
        super().__init__(controller, address=address, debug=debug)
    
        self.position1 = posA
        self.position2 = posB
        self.tolerance = tol
    
    def get_position(self):
        """Returns the position of the servo in PWM value"""
        code, addr, data = self.dict_command(shutter_,req="get_position")
        new_position = data[0]
        if code == "PO":
            if new_position != self.position:
                self.last_position = self.position
                self.position = new_position
            return new_position
        
        return None

    def set_position(self, data:int):
        """Sets and returns the position of the servo in PWM value"""
        code, addr, data = self.dict_command(shutter_,req="set_position",data=str(data))
        new_position = data[0]
        if code == "PO":
            if new_position != self.position:
                self.last_position = self.position
                self.position = new_position
                self.get_state()
            return new_position
        
        return None
    
    def get_state(self):
        """ Updates the current state of the shutter based on the preset PWM position values"""

        # Update position
        self.get_position()
        # Store last state
        self.last_state = self.state

        # Set state
        if self.position >= self.position1 - self.tolerance and self.position < self.position1 + self.tolerance:
            self.state = servo_states.A
        elif self.position >= self.position2 - self.tolerance and self.position < self.position2 + self.tolerance:
            self.state = servo_states.B
        else:
            self.state = None

        return self.state
    
    def set_state(self,newstate):
        if newstate == servo_states.A:
            self.set_position(data=str(self.position1))
            return self.get_state()
        elif newstate == servo_states.B:
            self.set_position(data=str(self.position2))
            return self.get_state()
        else:
            return None
    
    def flip(self):
        """Returns the position of the servo in PWM value"""
        
        if self.state == servo_states.B:
            self.set_position(data=str(self.position1))
            return self.get_state()
        elif self.state == servo_states.A:
            self.set_position(data=str(self.position2))
            return self.get_state()
        else:
            return None

default_config_dict = {
    "name": "SimpleShutter",
    "pos_A_name": "Power Meter",
    "pos_B_name": "Input",
    "servo_A": 120,
    "servo_B": 280,
    "servo_tol": 5,
    "sensor_open": 1,
    "servo_open": 1,
    "address_servo": 0,
    "address_sensor": 1,
}

class shutter_states(Enum):
    open = 1
    closed = 0
    mismatch = 2
    notdefined = 3

class ShutterWithSensor():

    def __init__(self, controller, config = default_config_dict, debug=False):

        self.servo = PwmServo(controller=controller, 
                              address=str(config["address_servo"]), 
                              posA=config["servo_A"],
                              posB=config["servo_B"],
                              tol=config["servo_tol"],
                              debug=debug)
        
        self.switch = SwitchSensor(controller=controller, 
                                   address=str(config["address_sensor"]), 
                                   openstate=config["sensor_open"],
                                   debug=debug)
        self.config = config
        self.state = None
        self.debug = debug

        self.get_state()

    def get_state(self):
        servo_state = self.servo.get_state()
        switch_state = self.switch.get_state()

        if servo_state and switch_state is not None:
            if servo_state == servo_states(self.config["servo_open"]) and switch_state == switch_states.open:
                self.state = shutter_states.open
            elif servo_state == servo_states(int(not(self.config["servo_open"]))) and switch_state == switch_states.closed:
                self.state = shutter_states.closed
            else:
                self.state = shutter_states.mismatch
        else:
            self.state = shutter_states.notdefined

        if self.debug:
            print(f'Servo position: {self.servo.get_position()}')
            print(f'Switch state:{self.switch.state}')
            print(self.state)

        return self.state
    
    def set_state(self,newstate):
        if newstate == shutter_states.open:
            self.servo.set_state(servo_states(self.config["servo_open"]))
        elif newstate ==shutter_states.closed:
            self.servo.set_state(shutter_states(int(not(self.config["servo_open"]))))

