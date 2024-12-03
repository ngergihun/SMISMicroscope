""" This file contains a dictionary of commands that the devices can accept in 
three different categories: get, set, move. For each command, it returns the 
instruction code to send to the device. """

from enum import Enum

get_ = {
    "position": "gp",
    "switchstate": "gs",
    "settings": "gv",
    "status": "mo"
}

set_ = {
    "mode": "mm",
    "targetspeed": "tv",
    "microsteps": "sm",
    "maxspeed": "sv",
    "maxacceleration": "sa",
    "switches": "es",
    "advancedstop": "as",
    "motorswitch": "ss",
    "position": "sp"
}

mov_ = {
    "absolute": "ma",
    "relative": "mr",
    "home": "mh",
    "autotune": "at",
}

class move_modes(Enum):
    soft = 0
    ramp = 1
    velocity = 2
    hold = 3

def commands():
    """Returns a dictionary of commands that the devices can accept."""
    return {"get": get_, "set": set_, "move": mov_}

if __name__ == "__main__":
    cmds = commands()
    print(cmds)
