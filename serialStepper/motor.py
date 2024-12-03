"""A module that contains the Motor class, which is the base class for all motors."""
from .cmd import get_, set_, mov_, move_modes
from .tools import error_check, move_check, is_null_or_empty
from .errors import ExternalDeviceNotFound

class BaseMotor:
    """A class that represents a general stepper motor motor. Each device inherits from this class."""
    def move(self):
        raise NotImplementedError
    
    def set_(self):
        raise NotImplementedError
    
    def get_(self):
        raise NotImplementedError

class Motor(BaseMotor):
    def __init__(self, controller, address="0", debug=False):
        super().__init__()
        # the controller object which services the COM port
        self.controller = controller
        self.address = address
        self.debug = debug

    def send_command(self, instruction, message=None):
        """Sends an instruction to the motor. Returns the response from the motor."""
        if message is not None:
            fullmessage = self.address + " " + message
        else:
            fullmessage = self.address
            
        cstatus = self.controller.send_instruction(instruction, message = fullmessage)
        status = self.translate_response(status=cstatus, debug=self.debug)
 
        return status
    
    def translate_response(self, status, debug=False):
        """Parses the message from the controller."""
        code = status[0]
        fulldata = status[1]
        if debug:
            print(f'Code: {code}')
            print(f'Data (str): {fulldata}')

        try:
            addr = int(fulldata[0])
        except ValueError as exc:
            raise ValueError(f"Invalid Address: {status[1]}.") from exc
        
        addr = int(fulldata[0])
        data = [int(d) for d in fulldata[1:]]

        if debug:
            print(f'Parsed Address: {addr}')
            print(f'Parsed Data: {data}')

        return (code, addr, data)
    
        # Basic functions
    
    def move(self, req="relative", data="0"):
        """Function initiate motor movement.

        Parameters
        ----------
        req : str, optinal
            Name of request
        data : str
            Parameters to be sent after address and request

        Returns
        -------
        status : tuple
            (code, addr, data)
        """

        if req in mov_:
            instruction = mov_[req]
        else:
            print(f"Invalid Command: {req}")
            return False

        status = self.send_command(instruction, message=data)

        return status
    
    def get(self, req="settings", data=""):
        """Generates the GET commands from get_ cmd dictionary.

        Parameters
        ----------
        req : str, optinal
            Name of request
        data : str
            Parameters to be sent after address and request

        Returns
        -------
        status : tuple
            (code, addr, data)
        """

        if req in get_:
            instruction = get_[req]
        else:
            print(f"Invalid Command: {req}")
            return None

        status = self.send_command(instruction, message=data)

        return status

    def set(self, req="", data=""):
        """Generates the SET commands from set_ cmd dictionary.

        Parameters
        ----------
        req : str, optinal
            Name of request
        data : str
            Parameters to be sent after address and request

        Returns
        -------
        status : tuple
            (code, addr, data)
        """

        if req in set_:
            instruction = set_[req]
        else:
            print(f"Invalid Command: {req}")
            return None

        status = self.send_command(instruction, message=data)

        return status

class CustomMotor(Motor):
    """Implement the stepper motor class for the SMIS motor controller"""

    def __init__(self, controller, address="0", debug=False, steps_per_rev=400):
        super().__init__(controller, address=address, debug=debug)

        self.last_dir = None
        self.position = None
        self.last_position = None

        # Important settings of the physical motor and controller
        self.steps_per_rev = steps_per_rev  # Step per revolution - physical property of the stepper motor
        self.microsteps_per_step = None # Microstep per step settings of the driver
        self.microsteps_per_rev = None
        self.max_velocity = None # Maximum velocity for the movement in ramp and soft mode
        self.max_acceleration = None # Maximum acceleration for ramp and soft mode
        self.target_velocity = 0 # Velocity in speed mode
        self.move_mode = move_modes.ramp
        self.is_moving = False

        self.get_position()
        self.get_settings()

    # Wrapper functions
    def move_relative(self, pos=0, wait=False):
        new_dir = bool(pos>=0)
        
        code, addr, data = self.move(req="relative",data=str(pos))

        if code == "MO":
            self.last_dir = new_dir
            moving = bool(data[0])
            if wait:
                while moving:
                    moving = self.get_status()

            return moving
        
        return None
    
    def move_absolute(self, pos=0, wait=False):
        self.get_position()
        new_dir = bool(pos>=self.position)

        code, addr, data = self.move(req="absolute",data=str(pos))

        if code == "MO":
            self.last_dir = new_dir
            moving = bool(data[0])
            if wait:
                while moving:
                    moving = self.get_status()

            return moving
        
        return None

    def get_position(self):
        """Returns the position of the motor in steps counted by the controller"""
        code, addr, data = self.get(req="position")
        new_position = data[0]
        if code == "PO":
            if new_position != self.position:
                self.last_position = self.position
            self.position = new_position
            return new_position
        
        return None

    def get_settings(self):
        """Gets the current settings of the motor and sets the corresponding object properties (microstepping, max velocity, max acceleration, target velocity)"""
        code, addr, data = self.get(req="settings")
        if code == "MV":
            self.microsteps_per_step = data[0]
            self.microsteps_per_rev = self.microsteps_per_step*self.steps_per_rev
            self.max_velocity = data[1]
            self.max_acceleration = data[2]
            self.target_velocity = data[3]
            return data

        return None
    
    def get_switch_states(self):
        """Returns the left and right switch states of the motor stage"""
        code, addr, data = self.get(req="switchstate")
        if code == "SS":
            Lswitch = data[0]
            Rswitch = data[1]
            return Lswitch, Rswitch
        
        return None

    def get_status(self):
        """Returns the current status of movement"""
        code, addr, data = self.get(req="status")
        if code == "MO":
            self.is_moving = bool(data[0])
            return bool(data[0])
        
        return None
    
    def set_position(self, pos = 0):
        """Set the steps reading of the controller to the given value"""
        code, addr, data = self.set(req="position",data=str(pos))
        position = data[0]
        if code == "PO":
            self.last_position = position
            self.position = position
            return position
        
        return None
    
    def set_switches(self, state = True):
        """Enables the default limit switch stops for all """
        code, addr, data = self.set(req="motorswitch",data=str(int(state)))
        if code == "SS":
            Lswitch = data[0]
            Rswitch = data[1]
            return Lswitch, Rswitch
        
        return None
    
    def set_mode(self, mode = move_modes.ramp):
        """Set the move mode of the motor. (soft, ramp, velocity, hold) See TMC429 datasheet"""
        code, addr, data = self.set(req="mode",data=str(mode.value))
        if code == "MM":
            self.move_mode = move_modes(data[0])
            return data[0]
        
        return None
    
    def set_microsteps(self, ms_per_step = 8):
        """Sets the microstepping of the motor"""
        microsteps_list = [1, 2, 4, 8, 16, 32, 64, 128, 256]
        
        if ms_per_step in microsteps_list:
            self.get_settings()
            oldpos = self.get_position()
            newpos = int(oldpos*(ms_per_step/self.microsteps_per_step))
            code, addr, data = self.set(req="microsteps", data=str(ms_per_step))
            if code == "MS":
                self.set_position(newpos)
                data = self.get_settings()
                return data[0]
            
            return None
        
        return None
    
    def set_maxvelocity(self, vmax = 6400):
        """Sets the maximum velocity to be reached in soft and ramp mode (in steps/sec)"""
        code, addr, data = self.set(req="maxspeed", data=str(vmax))
        if code == "VM":
            self.max_velocity = data[0]
            return data[0]
        
        return None
    
    def set_maxacceleration(self, amax = 12800):
        """Sets the maximum velocity to be reached in soft and ramp mode (in steps/sec)"""
        code, addr, data = self.set(req="maxacceleration", data=str(amax))
        if code == "AM":
            self.max_acceleration = data[0]
            return data[0]
        
        return None
    
    def set_targetvelocity(self, speed = 0):
        """Sets the target speed for the movement in velocity mode (in steps/sec, max 2047 steps/sec)"""
        code, addr, data = self.set(req="targetspeed", data=str(speed))
        if code == "VT":
            self.target_velocity = data[0]
            return data[0]
        
        return None

    def home(self):
        """Wrapper function to easily enable access to homing."""
        self.move("home")

    def autotune(self):
        """Wrapper function to easily enable access to homing."""
        code, addr, data = self.move(req="autotune",data=None)
        if code == "AT":
            return data

    def change_address(self, new_address):
        """Changes the address of the motor."""
        old_address = self.address
        status = self.set("address", data=new_address)
        if status[0] == new_address:
            # Make the Motor object know about the change
            self.address = new_address
            if self.debug:
                print(f"Address successfully changed from {old_address} to {new_address}.")
