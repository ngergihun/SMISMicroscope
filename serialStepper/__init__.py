"""The Elliptec Python Library"""
from .cmd import commands
from .errors import ExternalDeviceNotFound
from .scan import find_ports, scan_for_devices
from .tools import is_null_or_empty, error_check, move_check

# Classes for controllers
from .controller import BaseSerialController, Controller

# General class for all motors
from .motor import BaseMotor, Motor, CustomMotor

# Individual device implementations
# from .slider import Slider
# from .rotator import Rotator
from .linear import Linear
from .two_axis import TwoAxisStage
# from .iris import Iris

__all__ = [
    "commands",
    "devices",
    "ExternalDeviceNotFound",
    "BaseSerialController",
    "Controller",
    "BaseMotor",
    "Motor",
    "CustomMotor",
    "Linear",
    "TwoAxisStage",
    "find_ports",
    "scan_for_devices",
    "is_null_or_empty",
    "s32",
    "error_check",
    "move_check",
]
