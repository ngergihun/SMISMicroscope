"""Module for linear stages. Inherits from elliptec.Motor."""
from . import Linear
from . import linear
import math
import time

class TwoAxisStage:
    """Two axis stage class. Inherits from elliptec.Motor."""

    def __init__(self, controller, address_x="0", address_y="1", debug=False):
        self.xmotor = Linear(controller=controller, address=address_x, debug=False)
        self.ymotor = Linear(controller=controller, address=address_y, debug=False)

    ## Setting and getting positions

    def go_to_point(self, xdistance, ydistance, step_units=False, speed = 0.5, waiting=False):
        """Moves to the given position in 2D."""
        if not step_units:
            xsteps = linear.distance_to_steps(xdistance,
                                                threadpitch=self.xmotor.threadpitch,
                                                steps_per_rev=self.xmotor.microsteps_per_rev)
            ysteps = linear.distance_to_steps(ydistance,
                                                threadpitch=self.ymotor.threadpitch,
                                                steps_per_rev=self.ymotor.microsteps_per_rev)
        else:
            xsteps = xdistance
            ysteps = ydistance
        
        xstart = self.xmotor.get_position()
        ystart = self.ymotor.get_position()

        deltax = xsteps-xstart
        deltay = ysteps-ystart
        
        settings1 = self.xmotor.get_settings()
        settings2 = self.ymotor.get_settings()
        print("Settings1: ", settings1)
        print("Settings2: ", settings2)

        angle = math.atan2(deltay,deltax)
        v = math.sqrt(math.pow(settings1[1],2)+math.pow(settings2[1],2))
        a = math.sqrt(math.pow(settings1[2],2)+math.pow(settings2[2],2))

        speedx = abs(v*math.cos(angle))
        speedy = abs(v*math.sin(angle))
        accelx = abs(a*math.cos(angle))
        accely = abs(a*math.sin(angle))

        if speedx < 50:
            speedx = 50
            accelx = 200
        elif speedy < 50:
            speedy = 50
            accely = 200

        print("New Speedx: ", speedx, "New Speedy: ", speedy)

        # Set the calculated velocities
        self.xmotor.set_maxvelocity(vmax=int(speedx))
        self.ymotor.set_maxvelocity(vmax=int(speedy))
        # Set the calculated accelerations
        self.xmotor.set_maxacceleration(amax=int(accelx))
        self.ymotor.set_maxacceleration(amax=int(accely))
        # DO THE MOVEMENT
        self.xmotor.move_relative(pos=deltax, wait=False)
        self.ymotor.move_relative(pos=deltay, wait=False)
        
        if waiting is True:
            while self.is_moving():
                time.sleep(0.1)
            
            # Set back the original speed and accelaration
            self.xmotor.set_maxvelocity(vmax=settings1[1])
            self.ymotor.set_maxvelocity(vmax=settings2[1])
            self.xmotor.set_maxacceleration(amax=settings1[2])
            self.ymotor.set_maxacceleration(amax=settings2[2])
        

    def move(self, xdistance, ydistance, step_units=False, waiting=False):
        """Moves the stage with the defined distance in 2D."""
        if not step_units:
            xsteps = linear.distance_to_steps(xdistance,
                                                threadpitch=self.xmotor.threadpitch,
                                                steps_per_rev=self.xmotor.microsteps_per_rev)
            ysteps = linear.distance_to_steps(ydistance,
                                                threadpitch=self.ymotor.threadpitch,
                                                steps_per_rev=self.ymotor.microsteps_per_rev)
        else:
            xsteps = xdistance
            ysteps = ydistance

        settings1 = self.xmotor.get_settings()
        settings2 = self.ymotor.get_settings()

        angle = math.atan2(ysteps,xsteps)
        v = math.sqrt(math.pow(settings1[1],2)+math.pow(settings2[1],2))
        a = math.sqrt(math.pow(settings1[2],2)+math.pow(settings2[2],2))

        speedx = abs(v*math.cos(angle))
        speedy = abs(v*math.sin(angle))
        accelx = abs(a*math.cos(angle))
        accely = abs(a*math.sin(angle))

        # Set the calculated velocities
        self.xmotor.set_maxvelocity(vmax=int(speedx))
        self.ymotor.set_maxvelocity(vmax=int(speedy))
        # Set the calculated accelerations
        self.xmotor.set_maxacceleration(amax=int(accelx))
        self.ymotor.set_maxacceleration(amax=int(accely))
        # DO THE MOVEMENT
        self.xmotor.move_relative(pos=xsteps, wait=False)
        self.ymotor.move_relative(pos=ysteps, wait=False)
        
        if waiting is True:
            while self.is_moving():
                time.sleep(0.1)
            
            # Set back the original speed and accelaration
            self.xmotor.set_maxvelocity(vmax=settings1[1])
            self.ymotor.set_maxvelocity(vmax=settings2[1])
            self.xmotor.set_maxacceleration(amax=settings1[2])
            self.ymotor.set_maxacceleration(amax=settings2[2])

    def get_distance(self, distance):
        """Moves to a particular distance."""
        posx = self.xmotor.get_position()
        posy = self.ymotor.get_position()
        distancex = linear.steps_to_distance(posx,
                                             threadpitch=self.xmotor.threadpitch,
                                             steps_per_rev=self.xmotor.microsteps_per_rev)
        distancey = linear.steps_to_distance(posy,
                                             threadpitch=self.ymotor.threadpitch,
                                             steps_per_rev=self.ymotor.microsteps_per_rev)
        
        return distancex, distancey
    
    def is_moving(self):
        """Checks if any of the motors are moving"""
        xmove = self.xmotor.get_status()
        ymove = self.ymotor.get_status()

        return xmove or ymove

