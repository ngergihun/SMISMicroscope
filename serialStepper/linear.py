"""Module for linear stages. """
from . import CustomMotor


class Linear(CustomMotor):
    """Linear Motor class. Inherits from Motor."""

    def __init__(self, controller, address="0", debug=False):
        super().__init__(controller=controller, address=address, debug=debug)

        self.threadpitch = 1.0 # mm/rev
        self.wait_movement = False

    ## Setting and getting positions

    def move_to(self, distance):
        """Moves to a particular distance."""
        steps = distance_to_steps(distance,threadpitch=self.threadpitch,steps_per_rev=self.microsteps_per_rev)
        status = self.move_absolute(pos=steps, wait=self.wait_movement)
        
        return status

    def move_distance(self, distance):
        """Shifts by a particular distance."""
        steps = distance_to_steps(distance,threadpitch=self.threadpitch,steps_per_rev=self.microsteps_per_rev)
        status = self.move_relative(pos=steps, wait=self.wait_movement)
        
        return status
    
    def get_distance(self):
        """Moves to a particular distance."""
        pos = self.get_position()
        distance = steps_to_distance(pos,threadpitch=self.threadpitch,steps_per_rev=self.microsteps_per_rev)
        
        return distance

    def jog(self, direction="forward", speed=0.0):
        """Jogs by the jog speed in a particular direction."""
        speed_steps = int(distance_to_steps(speed,threadpitch=self.threadpitch,steps_per_rev=self.microsteps_per_rev))
        
        if speed_steps > 2047:
            speed_steps = 2047
        
        if direction in ["backward", "forward"]:
            if direction == "backward":
                self.set_targetvelocity(speed=-1*speed_steps)
                status = self.get_status()
                return status
            else:
                self.set_targetvelocity(speed=speed_steps)
                status = self.get_status()
                return status
        else:
            return None
        
    # Set the speed and acceleration
    def set_speed(self, speed, threadpitch=1.0, steps_per_rev=3200):
        """Sets the motor speed in microns per seconds."""
        step_speed = int(speed/(threadpitch*1000)*steps_per_rev)
        self.set_maxvelocity(vmax=step_speed)
        self.set_maxacceleration(amax=4*step_speed)

# Helper functions

def steps_to_distance(steps, threadpitch=1.0, steps_per_rev=3200):
    """Converts position in pulses to distance in millimeters."""
    distance = steps/steps_per_rev*threadpitch*1000
    return distance

def distance_to_steps(distance, threadpitch=1.0, steps_per_rev=3200):
    """Converts distance in microns to steps."""
    revs = distance / (threadpitch*1000)
    steps = int(revs*steps_per_rev)

    # moved_dist = steps/steps_per_rev*threadpitch*1000

    return steps

def calc_threadpitch(steps, distance, steps_per_rev=3200):
    """Calculates the threadpitch value from steps-distance data"""
    revs = steps/steps_per_rev

    threadpitch = distance/revs/1000

    return threadpitch