"""Method for scanner for a specific device attached to a COM port"""

import serial as s
import serial.tools.list_ports as listports
from .errors import ExternalDeviceNotFound
from .motor import Motor

def find_ports():
    """Find all available ports with a device connected."""
    available_ports = []
    for port in listports.comports():
        if port.serial_number:
            try:
                connection = s.Serial(port.device)
                connection.close()
                available_ports.append(port)
            except (OSError, s.SerialException):
                print(f"{port.device} unavailable.\n")
    port_names = [port.device for port in available_ports]
    return port_names


def scan_for_devices(controller, start_address=0, stop_address=0, debug=True):
    """Scan for devices on a controller. Returns a list of dictionaries with device info and controller object."""
    devices = []
    for address in range(start_address, stop_address + 1):
        try:
            motor = Motor(controller, address=str(address), debug=debug)
            print(f"{controller.port}, address {address}: ELL{motor.motor_type} \t(S/N: {motor.serial_no})")
            device = {
                "info": motor.info,
                "controller": controller,
            }
            devices.append(device)
            del motor
        except ExternalDeviceNotFound:
            pass
    return devices
