
class BaseDevice:
    """A class that represents a general stepper motor motor. Each device inherits from this class."""
    def send_command(self):
        raise NotImplementedError
    
    def translate_response(self):
        raise NotImplementedError

class DeviceWithAddress(BaseDevice):
    """A class that represents a general stepper motor motor. Each device inherits from this class."""
    def __init__(self, controller, address="0", debug=False):
        super().__init__()
        # the controller object which services the COM port
        self.controller = controller
        self.address = address
        self.debug = debug

    def dict_command(self, dict_source, req="get_position", data=""):
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

        if req in dict_source:
            instruction = dict_source[req]
            print(instruction)
        else:
            print(f"Invalid Command: {req}")
            return None

        status = self.send_command(instruction, message=data)

        return status

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