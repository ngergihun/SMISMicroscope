"""This module contains the Controller class, which is the base class for all devices."""

import sys
import serial
from serial.serialutil import PARITY_NONE, STOPBITS_ONE
from .tools import is_null_or_empty

class BaseSerialController:
    """
    Base class controlling stepper motor controller with serial interface.
    """

    def __init__(self,port,baudrate=9600,bytesize=8,parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,timeout=10,write_timeout=0.5,debug=True):
        
        self.debug = debug
        self.port = port

        self.last_response = None
        self.last_status = None
    
        try:
            self.s = serial.Serial(
                port,
                baudrate=baudrate,
                bytesize=bytesize,
                parity=parity,
                stopbits=stopbits,
                timeout=timeout,
                write_timeout=write_timeout,
            )
        except serial.SerialException:
            print("Could not open port {port}.")
            self.s = None

        if self.s.is_open:
            if self.debug:
                print(f"Controller on port {port}: Connection established!")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.s.close()

    def read_response(self):
        """
        Read the response of the controller received from the serial port
        """
        raise NotImplementedError
    
    def parse(self):
        """
        Parses the response from the serial port to values
        """
        raise NotImplementedError
    
    def send_to_serial(self,fullmessage):
        """
        Send the created message through the serial device and listens to the response if enabled
        """
        raise NotImplementedError

    def send_instruction(self, instruction, message = None):
        """
        Creates the complex command message, transmits the message to the serial device and listens to the answer which is then parse to a useble format
        """
        raise NotImplementedError

    def close_connection(self):
        """Closes the serial connection."""
        if self.s.is_open:
            self.s.close()
            print("Connection is closed!")


class Controller(BaseSerialController):
    """Implements a general serial controller with an immediet response from the device"""
    def __init__(self, port, baudrate=9600, bytesize=8, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=10, write_timeout=0.5, debug=True,
                 end_char="\r\n", response_separator=" ", request_separator=" ", data_separator=" "):
        super().__init__(port, baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits, timeout=timeout, write_timeout=write_timeout, debug=debug)

        self.end_char = end_char
        self.response_separator = response_separator
        self.request_separator = request_separator
        self.data_separator = data_separator

    def read_response(self):
        """Reads the response from the controller.
        
        Reads the last response message of the 
        
        """
        response = self.s.read_until(self.end_char.encode())  # Waiting until response read

        if self.debug:
            print("RX:", response.strip())

        return response
    
    def parse(self, response, debug=True):
        """Parses the received answer/message.
        
        The basic implementation expects a code string to arrive with the message from the serial device. Reimplement according to your on device communication protocols.

        Parameters
        ----------
        answer : bytes
            Answer/message from the serial device to be parsed.
        debug : bool, optional
            Turns on/off debug messages
        """
        if is_null_or_empty(response):
            if debug:
                print("Parse: Status/Response may be incomplete!")
                print("Parse: Message:", response)
            return None
        
        response = response.decode().strip().split(sep=self.response_separator) # this is the full message from the serial port
        code = response[0] # code- usually string defines the data meaning
        
        if self.debug:
            print(f'Code: {code}')
            print(f'Data: {response[1:]}')

        return (code, response[1:])

    def create_message_from_list(self, data_list):
        """
        Creates the message from a list of data to be sent

        Parameters
        ----------
        data_list : list
            List containing the data pieces to be sent after the command character.
        """
        if data_list is not None:
            msg = str()
            for d in data_list:
                try:
                    for d in data_list:
                        msg+=(self.data_separator)
                        msg+=(str(d))
                    return msg
                except:
                    return None
        else:
            return None

    def send_to_serial(self, code=None, message=None):
        """Sends an instruction to the controller. Expects a response which is returned."""
        # self.s.reset_input_buffer() # NOT SURE if we need this
        # Compose the message if it is a list
        if type(message) is list:
            message = self.create_message_from_list(message)
        # Start with the command is necessary
        if code is not None:
            command = code + self.request_separator
        # Append command if necessary
        if message is not None:
            mesg = str(message)
            command += mesg + '\n'
            command = command.encode()

        if command:
            if self.debug:
                print("TX:", command)
            # Execute the command and wait for a response
            self.s.write(command)  
            # Read the response of the serial port
            response = self.read_response()
            return response
        else:
            return None
    
    def send_instruction(self, instruction: str, message = None):
        """Wrapper function that sends the instruction with the data message. Reads the last response and parses it into the specified format.
        
        Parameters
        ----------
        instruction : str
            Instruction code for the serial device to command a specific behaviour.
        message : list, str, optinal
            Describes the data message sent following the instruction code

        Returns
        -------
        status : tuple
            (code, response data) code is the response code of the device for a specific request which puts the following response data into context.
        """
        response = self.send_to_serial(instruction, message)
        status = self.parse(response=response, debug=self.debug)

        self.last_response = response
        self.last_status = status
        if self.debug:
            print('STATUS:', status)

        return status
