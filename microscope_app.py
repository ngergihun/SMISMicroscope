# -*- coding: utf-8 -*-
"""
GUI app for home-built microscope with OSC (open stepper controller) motor controller and IDS camera
Standalone app
@author: Gergely Németh
"""

#General modules
import sys
import os
import numpy as np
from enum import Enum
from time import sleep
import logging
import yaml

# GUI modules
from PySide6 import QtWidgets, QtGui
from PySide6 import QtCore
from PySide6.QtCore import QObject, QThread, Signal, Slot, QTimer
import pyqtgraph as pg
pg.setConfigOption('useNumba', True)
pg.setConfigOption('imageAxisOrder', 'row-major')
from qt_material import apply_stylesheet

# Logging config
logging.basicConfig(level=logging.INFO)

# Stepper controller package
import serialStepper
from serialStepper.cmd import move_modes
from serialStepper import linear

class units(Enum):
    steps = 0
    microns = 1
class step_mode(Enum):
    relative = 0
    absolute = 1

# Joystick package
import pyspacemouse

# Camera packages
from ids_peak import ids_peak
from ids_peak_ipl import ids_peak_ipl
from ids_peak import ids_peak_ipl_extension

from camera.camera import IDS_Camera
from skimage.registration import phase_cross_correlation
from skimage.color import rgb2gray, rgba2rgb

FPS_LIMIT = 50
# TARGET_PIXEL_FORMAT = ids_peak_ipl.PixelFormatName_RGBa8
TARGET_PIXEL_FORMAT = ids_peak_ipl.PixelFormatName_RGB8

# UI import
ui_file_name = 'microscope_app.ui'
current_folder = os.getcwd()
ui_file_path = os.path.join(current_folder,ui_file_name)

uiclass, baseclass = pg.Qt.loadUiType(ui_file_path)

class CameraThread(QObject):
    """Qt thread handling the camera reading independently from the UI"""
    progress = Signal()

    def __init__(self):
        super().__init__()

        try:
            self.camera = IDS_Camera()
        except:
            self.camera = None

        if self.camera.open_device():
            logging.info(f"Camera: {self.camera.name}")
        else:
            self.camera.destroy_all()
            logging.critical("Could not connect to camera")

        # Timers
        self.__init_timers()
        # Flags set by the UI thread
        self.__freerun_enabled = False
        # Other
        self.__freerun_error_counter = 0

    @Slot()
    def start_freerun(self):
        """
        Start Acquisition on camera and start the acquisition timer to receive and display images

        :return: True/False if acquisition start was successful
        """
        # Check that a device is opened and that the acquisition is NOT running. If not, return.
        if self.camera.device is None:
            return False

        # Get the maximum framerate possible, limit it to the configured FPS_LIMIT. If the limit can't be reached, set
        # acquisition interval to the maximum possible framerate
        
        if self.camera.set_continuous():
            self.__freerun_enabled = True
            self.__freerun_timer.setInterval(int(1 / self.camera.framerate) * 1000)
            self.__freerun_timer.start()
            logging.info("Freerun started")
            return True
        else:
            logging.error("Could not start acquisition!")
            return False
   
    @Slot()
    def stop_freerun(self):
        """
        Stop acquisition timer and stop acquisition on camera
        :return:
        """
        logging.info("Freerun stopped")
        self.__freerun_timer.stop()
        self.camera.stop_acquisition()
        self.__freerun_enabled = False
    
    @Slot()
    def on_exposure_change(self,value):
        # self.__stop_acquisition()
        newfps = round(1000000/value,1)
        self.camera.set_framerate(newfps)
        self.camera.set_exposuretime(value)
            
        # Setup acquisition timer accordingly
        self.__freerun_timer.setInterval((1 / self.camera.framerate) * 1000)

    def __init_timers(self):

        self.__freerun_timer = QTimer(self)
        self.__freerun_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.__freerun_timer.timeout.connect(self.__on_freerun_timer)

    def __on_freerun_timer(self):
        """
        This function gets called on every timeout of the freerun timer
        """
        try:
            self.image = self.camera.get_image()
            self.progress.emit()
        except ids_peak.Exception as e:
            self.__freerun_error_counter += 1
            logging.error("Could not get image: " + str(e))

    def __del__(self):
        self.camera.destroy_all()

class MotorThread(QObject):
    """Qt thread handling the motor control (including joystick handling) independently from the UI"""
    pos_updated = Signal()

    def __init__(self, port="COM10", address_x="0", address_y="1",debug=False):
        super().__init__()

        self.stageport = port
        self.xpos = None
        self.ypos = None
        self.zpos = 0
        self.moving = False
        self.target_xmove = 0
        self.target_ymove = 0
        self.target_zmove = 0
        self.unit = units.steps

        self.xthreadpitch = 0.5039370078740157/1.008
        self.ythreadpitch = 0.5039370078740157/1.008

        self.x_enabled = True
        self.y_enabled = True
        self.z_enabled = False

        self.speed_range = "Low"

        # Private objects
        self.__stage_controller = None
        self.__stage = None
        self.__button_motor = None

        # Private flags
        self.__joystick_enabled = False
        self.__success_mouse_open = None

        # Private settings
        self.__joystick_poll_time = 1

        self.is_stage_available = False
        self.is_stage_available = self.__init_stage(port=self.stageport,address_x=address_x,address_y=address_y,debug=False)

        logging.info(f"Stage ready? {self.is_stage_available}")

        if self.is_stage_available:
            self.__init_timers()

    def __init_stage(self, port="COM10", address_x="0",address_y="1",debug=False):
        # Initialize the stage controller
        try:
            self.__stage_controller = serialStepper.Controller(port, debug=False)
            self.__stage = serialStepper.TwoAxisStage(controller=self.__stage_controller, 
                                                    address_x=address_x, 
                                                    address_y=address_y,
                                                    debug=debug)
            self.__stage.xmotor.threadpitch = self.xthreadpitch
            self.__stage.ymotor.threadpitch = self.ythreadpitch

            for motor in [self.__stage.xmotor, self.__stage.ymotor]:
                self.__set_stage_speed(motor, speed=0.5)
            
            logging.info("Stage setup succesfull!")
            return True
        except:
            return False
        
    def __init_timers(self):
        self.__timer_joystick = QTimer(self)
        self.__timer_joystick.setTimerType(QtCore.Qt.PreciseTimer)
        self.__timer_joystick.timeout.connect(self.__read_joystick)

        self.__timer_pos_update = QTimer(self)
        self.__timer_pos_update.setTimerType(QtCore.Qt.PreciseTimer)
        self.__timer_pos_update.timeout.connect(self.__read_position)
        self.__timer_pos_update.start(100)

    def __read_position(self):
        if self.unit == units.steps:
            self.xpos = self.__stage.xmotor.get_position()
            self.ypos = self.__stage.ymotor.get_position()
        elif self.unit == units.microns:
            self.xpos = self.__stage.xmotor.get_distance()
            self.ypos = self.__stage.ymotor.get_distance()

        self.moving = self.__stage.is_moving()

        self.pos_updated.emit()

    def __set_stage_speed(self, motor, speed=1):
        newv = int(motor.microsteps_per_rev * speed)
        newa = newv * 4
        if self.__joystick_enabled:
            motor.set_mode(move_modes.ramp)
        motor.set_maxvelocity(vmax=newv)
        motor.set_maxacceleration(amax=newa)
        if self.__joystick_enabled:
            motor.set_mode(move_modes.velocity)

    def __read_joystick(self):
        if self.__joystick_enabled:
            state = pyspacemouse.read()
            if state is not None:
                if self.x_enabled:
                    self.__stage.xmotor.set_targetvelocity(speed=int(state.x*2047))

                if self.y_enabled:
                    self.__stage.ymotor.set_targetvelocity(speed=int(state.y*2047))

                if self.z_enabled:
                    pass

    def __activate_pointmove(self):
        self.__timer_joystick.stop()
        self.__joystick_enabled = False
        if self.__success_mouse_open is not None:
            self.__success_mouse_open.close()

        for motor in [self.__stage.xmotor, self.__stage.ymotor]:
            motor.set_mode(move_modes.ramp)
        logging.info("Set Ramp mode")

    def __activate_velocitymove(self):
        self.__timer_joystick.start(self.__joystick_poll_time)
        self.__joystick_enabled = True
        try:
            self.__success_mouse_open = pyspacemouse.open()
            self.on_speed_selection_changed(self.speed_range)
        except:
            self.__success_mouse_open = None
            logging.warning("Warning Could not open 3D mouse!")
        if self.__success_mouse_open is not None:
            # Set the motors to velocity mode
            for motor in [self.__stage.xmotor, self.__stage.ymotor]:
                motor.set_mode(move_modes.velocity)
            logging.info("Set Velocity mode")
        else:
            logging.error("Warning: Could access 3D mouse!")

    @Slot()
    def move_to_point(self,relative=True,stepunit=True):
        """ Moves the stage to a relative or absolute coordinate specified in either step or real units"""

        if relative is False:
            self.__stage.go_to_point(xdistance=self.target_xmove,
                                   ydistance=self.target_ymove,
                                   step_units=stepunit)
        elif relative is True:
            self.__stage.move(xdistance=self.target_xmove,
                            ydistance=self.target_ymove,
                            step_units=stepunit)
        else:
            logging.info("Movement definition is not clear!")

        self.__read_position()
        # moving_timer = QTimer()
        # moving_timer.setSingleShot(True)
        # moving_timer.timeout.connect(self.__read_position)

        # self.__read_position()
        # while self.moving:
        #     moving_timer.start(10)
        #     # pass
        # print("Arrived")
        
    @Slot()
    def reset_position(self):
        """ Slot: Set the current position of the stage motors to ZERO"""
        for motor in [self.__stage.xmotor, self.__stage.ymotor]:
            motor.set_position(pos=0)
            if self.__joystick_enabled:
                motor.set_mode(mode=move_modes.velocity)
        logging.info("Current position was reset to zero!")

    @Slot()
    def on_position_update(self):
        self.__read_position()
        self.pos_updated.emit()
    
    @Slot()
    def on_control_mode_change(self, tabindex):
        if tabindex == 0:
            self.__activate_pointmove()

        elif tabindex == 1:
            self.__activate_velocitymove()

    @Slot()
    def on_speed_selection_changed(self, buttontext):
        if buttontext == "Low":
            for motor in [self.__stage.xmotor, self.__stage.ymotor]:
                self.__set_stage_speed(motor, speed=0.5)               

        elif buttontext == "Mid":
            for motor in [self.__stage.xmotor, self.__stage.ymotor]:
                self.__set_stage_speed(motor, speed=1)

        elif buttontext == "High":
            for motor in [self.__stage.xmotor, self.__stage.ymotor]:
                self.__set_stage_speed(motor, speed=2)

        self.speed_range = buttontext

    @Slot()
    def on_button_move(self,button_name):
        self.__joystick_enabled = False
        if button_name == "y_up":
            self.__button_motor = self.__stage.ymotor
            self.__button_motor.set_targetvelocity(speed=int(2047))
        elif button_name == "y_down":
            self.__button_motor = self.__stage.ymotor
            self.__button_motor.set_targetvelocity(speed=int(-1*2047))
        elif button_name == "x_right":
            self.__button_motor = self.__stage.xmotor
            self.__button_motor.set_targetvelocity(speed=int(2047))
        elif button_name == "x_left":
            self.__button_motor = self.__stage.xmotor
            self.__button_motor.set_targetvelocity(speed=int(-1*2047))
        elif button_name == "":
            self.__button_motor.set_targetvelocity(speed=int(0))
            self.__joystick_enabled = True
            
class MicroscopeApp(uiclass, baseclass):
    # Motor signals
    move_command_signal = Signal(bool,bool)
    motorspeed_change_signal = Signal(str)
    button_move_command = Signal(str)

    # Camera signals
    camera_start_signal = Signal()
    camera_stop_signal = Signal()
    set_exposure_signal = Signal(int)

    def __init__(self):
        super().__init__()

    # Initialize the user interface from the generated module
        self.setupUi(self)
        # apply_stylesheet(app, theme='dark_blue.xml', invert_secondary=True)

    # Flags, attributes and config
        self.load_config()
        self.objective_combo.addItems(self.config["objectives"].keys())
        self.microns_per_pixel = self.config["objectives"][self.objective_combo.currentText()]["pixel_to_distance"]
        self.steps_per_pixel = None
        self.moving = False
        self.click_move_enabled = False
        self.fixed_image = None

    # Create the CAMERA worker thread
        self.camera_worker = CameraThread()
        self.camera_thread = QThread()
        self.camera_worker.progress.connect(self.update_image)
        self.camera_worker.moveToThread(self.camera_thread)
        self.camera_thread.start()

    # Create the STAGE CONTROLLER worker thread
        port = self.config['stage']['port']
        self.motor_worker = MotorThread(port=port, address_x="1", address_y="0")
        self.motor_thread = QThread()
        self.motor_worker.moveToThread(self.motor_thread)
        self.motor_thread.start()

    # Timer to locally monitor the movement
        self.moving_timer = QTimer()
        self.moving_timer.setTimerType(QtCore.Qt.PreciseTimer)
        # self.moving_timer.timeout.connect(self.get_movement)

    # SIGNAL connects to MOTOR related command signals and buttons
        self.objective_combo.currentIndexChanged.connect(self.objective_change)
        self.click_move_button.clicked.connect(self.click_move_enable)
        if self.motor_worker.is_stage_available:
            # Position update
            self.motor_worker.pos_updated.connect(self.position_label_update)
            self.motor_worker.pos_updated.connect(self.update_image_scales)
            # Tab change to control mode change (joystick/velocity or pointmove/ramp mode)
            self.control_tab.currentChanged.connect(self.motor_worker.on_control_mode_change)
            # Speed switch radiobuttons
            self.low_speed_radio.toggled.connect(lambda: self.speed_selection_changed(self.low_speed_radio))
            self.mid_speed_radio.toggled.connect(lambda: self.speed_selection_changed(self.mid_speed_radio))
            self.high_speed_radio.toggled.connect(lambda: self.speed_selection_changed(self.high_speed_radio))
            # Unit switch radiobuttons
            # self.steps_radio.toggled.connect(lambda: self.distance_unit_changed(self.steps_radio))
            # self.microns_radio.toggled.connect(lambda: self.distance_unit_changed(self.microns_radio))
            # Move and position reset buttons
            self.move_button.clicked.connect(self.move_to_buttoncall)
            self.move_command_signal.connect(self.motor_worker.move_to_point)
            self.reset_pos_button.clicked.connect(self.motor_worker.reset_position)
            self.motorspeed_change_signal.connect(self.motor_worker.on_speed_selection_changed)
            # Connect move button signals
            self.y_up.pressed.connect(lambda: self.movebutton_on_click(self.y_up))
            self.y_up.released.connect(lambda: self.movebutton_on_release(self.y_up))
            self.y_up.setShortcut(QtGui.QKeySequence.MoveToPreviousLine)
            self.y_down.pressed.connect(lambda: self.movebutton_on_click(self.y_down))
            self.y_down.released.connect(lambda: self.movebutton_on_release(self.y_down))
            self.y_down.setShortcut(QtGui.QKeySequence.MoveToNextLine)
            self.x_right.pressed.connect(lambda: self.movebutton_on_click(self.x_right))
            self.x_right.released.connect(lambda: self.movebutton_on_release(self.x_right))
            self.x_right.setShortcut(QtGui.QKeySequence.MoveToPreviousChar)
            self.x_left.pressed.connect(lambda: self.movebutton_on_click(self.x_left))
            self.x_left.released.connect(lambda: self.movebutton_on_release(self.x_left))
            self.x_left.setShortcut(QtGui.QKeySequence.MoveToNextChar)
            # Connect axis lock checkboxes
            self.x_lock_switch.stateChanged.connect(self.on_axis_lock_change)
            self.y_lock_switch.stateChanged.connect(self.on_axis_lock_change)
            self.z_lock_switch.stateChanged.connect(self.on_axis_lock_change)
            # calibrate
            self.calibrate_button.clicked.connect(self.calibrate_pixels)

            self.unit = units.microns
            self.motor_worker.unit = units.microns

            self.control_tab.currentChanged.emit(self.control_tab.currentIndex())

    # CAMERA CONTROL UI ITEMS
        self.exposure_slider.valueChanged.connect(self.set_exposure)
        self.set_exposure_signal.connect(self.camera_worker.on_exposure_change)
        self.gain_slider.valueChanged.connect(self.set_gain)
        self.camera_start_button.clicked.connect(self.start_stop_camera)
        self.camera_start_signal.connect(self.camera_worker.start_freerun)
        self.camera_stop_signal.connect(self.camera_worker.stop_freerun)
        self.register_image_button.clicked.connect(self.register_image_to_map)

    # IMAGE DISPLAY AREA SETUP
        self.customize_display()
        self.imItem = pg.ImageItem(np.zeros((2076,3088,4), dtype="uint8"), autoLevels=False)             # create an ImageItem
        self.image_view_area.addItem(self.imItem)                                                   # add it to the PlotWidget
        self.imItem.hoverEvent = self.imageHoverEvent
        self.imItem.mouseClickEvent = self.imageClickEvent
        
        tr = QtGui.QTransform()                                                                   # prepare ImageItem transformation:
        tr.translate(-3088/2*self.microns_per_pixel,-2076/2*self.microns_per_pixel)               # move 3x3 image to locate center at axis origin
        tr.scale(self.microns_per_pixel,self.microns_per_pixel)                                    # scale horizontal and vertical axes
        self.imItem.setTransform(tr)

# IMAGE DISPLAY CUSTOMIZATION
    def customize_display(self):
        self.image_view_area.setBackground('w')
        self.image_view_area.setAspectLocked(True)
        self.image_view_area.invertY(True)
        self.image_view_area.getAxis('left').setTextPen('black')
        self.image_view_area.getAxis('bottom').setTextPen('black')
        self.crosshair_vLine = pg.InfiniteLine(angle=90, movable=False)
        self.crosshair_hLine = pg.InfiniteLine(angle=0, movable=False)
        self.image_view_area.addItem(self.crosshair_vLine, ignoreBounds=True)
        self.image_view_area.addItem(self.crosshair_hLine, ignoreBounds=True)
        self.image_view_area.getAxis('bottom').setLabel('X position / μm')
        self.image_view_area.getAxis('left').setLabel('Y position / μm')
        self.image_view_area.showAxes(True)
        self.image_view_area.setAspectLocked(True)
        self.image_view_area.setMouseTracking(True)

# MOTOR RELATED FUNCTIONS
    def get_movement(self):
        return self.motor_worker.moving
    
    def position_label_update(self):
        """ Gets the position from the motor thread and updates the labels"""
        if self.motor_worker.unit == units.steps:
            unit_string = ""
        elif self.motor_worker.unit == units.microns:
            unit_string = "μm"
        self.xpos_label.setText(f'X: {round(self.motor_worker.xpos,2)} {unit_string}')
        self.ypos_label.setText(f'Y: {round(self.motor_worker.ypos,2)} {unit_string}')
        self.zpos_label.setText(f'Z: {round(self.motor_worker.zpos,2)} {unit_string}')
        
    def move_to_buttoncall(self):
        self.motor_worker.target_xmove = self.xmove_spinbox.value()
        self.motor_worker.target_ymove = self.ymove_spinbox.value()
        self.motor_worker.target_zmove = self.zmove_spinbox.value()

        unitbool = (self.motor_worker.unit == units.steps)

        if self.relative_move_radio.isChecked():
            self.move_command_signal.emit(True,unitbool)

        if self.absolute_move_radio.isChecked():
            self.move_command_signal.emit(False,unitbool)

    def distance_unit_changed(self, button):
        if button.isChecked():
            if button.text() == "steps":
                self.motor_worker.unit = units.steps
            elif button.text() == "microns":
                self.motor_worker.unit = units.microns
            else:
                pass

    def speed_selection_changed(self, button):
        if button.isChecked():
            # self.speed_range = button.text()
            self.motorspeed_change_signal.emit(button.text())

    def movebutton_on_release(self, button):
        """ Turns off the motor moved by the buttons"""
        # self.
        self.button_move_command.emit("")

    def movebutton_on_click(self, button):
        self.button_move_command.emit(button.objectName())

    def on_axis_lock_change(self):
        self.motor_worker.x_enabled = self.x_lock_switch.isChecked()
        self.motor_worker.y_enabled = self.y_lock_switch.isChecked()
        self.motor_worker.z_enabled = self.z_lock_switch.isChecked()

    def imageClickEvent(self, event):
        # Get mouse position
        pos = event.pos()
        ppos = self.imItem.mapToParent(pos)
        self.click_x, self.click_y = ppos.x(), ppos.y()

        if self.click_move_enabled:
            self.click_move()

    def click_move(self):
        """ Initiates position movement by clicking on the image"""
        self.motor_worker.target_xmove = -self.click_x
        self.motor_worker.target_ymove = self.click_y
        
        logging.info(f"X pixel coordinate: {self.click_x}, Y pixel coordinate {self.click_y}")
        
        # self.motorspeed_change_signal.emit(self.motor_worker.speed_range)
        self.move_command_signal.emit(False,False)
        self.click_move_enabled = False

        sleep(0.1)
        self.moving_timer = QTimer()
        self.moving_timer.setTimerType(QtCore.Qt.PreciseTimer)
        # self.moving_timer.timeout.connect(lambda: self.run_when_stopped(self.motorspeed_change_signal.emit, self.motor_worker.speed_range))
        self.moving_timer.timeout.connect(lambda: self.run_when_stopped(self.click_move_end))
        self.moving_timer.start(200)

    def click_move_end(self):
        self.motorspeed_change_signal.emit(self.motor_worker.speed_range)
        self.click_move_enabled = True
        
    def imageHoverEvent(self,event):
        # Show the position, pixel, and value under the mouse cursor.
        if event.isExit():
            self.image_view_area.setTitle("")
            return
        pos = event.pos()
        i, j = pos.y(), pos.x()
        # i = int(np.clip(i, 0, self.imItem.image.shape[0] - 1))
        # j = int(np.clip(j, 0, self.imItem.image.shape[1] - 1))
        # val = self.imItem.image[i, j]
        ppos = self.imItem.mapToParent(pos)
        x, y = ppos.x(), ppos.y()
        # self.image_view_area.setTitle("pos: (%0.1f, %0.1f)  pixel: (%d, %d)  value: %.3g" % (x, y, i, j, val))
        self.image_view_area.setTitle("pos: (%0.1f, %0.1f)  pixel: (%d, %d)"  % (x, y, i, j))

    def click_move_enable(self):
        if self.click_move_button.isChecked():
            self.click_move_enabled = True
        else:
            self.click_move_enabled = False

# MOTOR AND CAMERA RELATED FUNCTIONS
    def update_image_scales(self):
        tr = self.imItem.transform()

        newdx = -3088/2*self.microns_per_pixel-self.motor_worker.xpos
        newdy = -2076/2*self.microns_per_pixel+self.motor_worker.ypos

        if self.motor_worker.unit == units.microns:
            tr.reset()
            tr.translate(newdx,newdy)
            tr.scale(self.microns_per_pixel,self.microns_per_pixel)
            self.imItem.setTransform(tr)
            self.crosshair_vLine.setPos(self.motor_worker.xpos*-1.0)
            self.crosshair_hLine.setPos(self.motor_worker.ypos)
        elif self.motor_worker.unit == units.steps:
            pass

    def calibrate_pixels(self):
        self.start_stage_calibration(xsteps=-1000,ysteps=0)
        
    def start_stage_calibration(self, xsteps, ysteps):
        button = QtWidgets.QMessageBox.question(self,"Question dialog","Do you want to proceed?")
        if button == QtWidgets.QMessageBox.Yes:
            pass
        else:
            return
        
        # Grab an image
        # NOTE: Do it smarter, dont check the button, but the camera itself
        if self.camera_start_button.isChecked():
            image1 = rgb2gray(rgba2rgb(self.camera_worker.camera.image))
            logging.info("First image captured!")
        else:
            return
        
        # Move
        # sleep(0.1)
        if self.motor_worker.is_stage_available:
            self.motor_worker.target_xmove = xsteps
            self.motor_worker.target_ymove = ysteps

            if self.control_tab.currentIndex() == 1:
                self.control_tab.setCurrentIndex(0)
                sleep(0.2)
            
            self.move_command_signal.emit(True,True)
        else:
            logging.error("Stage is not initialized!")
            return

        sleep(0.1)
        self.moving_timer = QTimer()
        self.moving_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.moving_timer.timeout.connect(lambda: self.run_when_stopped(self.end_stage_calibration, image1 = image1))
        self.moving_timer.start(200)

    def end_stage_calibration(self, image1 = None):
        """ Calculates the angle between camera and motor axes. Called when motor stopped after movement. """
        # Grab another image when arrived
        image2 = rgb2gray(rgba2rgb(self.camera_worker.camera.image))
        # Calculate shift

        shift, _, _ = phase_cross_correlation(image1, image2)
        logging.info(f"Pixel shift: X={shift[1]}, Y={shift[0]}")

        angle = np.arctan(np.divide(shift[0],shift[1]))
        logging.info(f"Angle between camera and motor axes: {np.rad2deg(angle)}")
        # self.steps_per_pixel = shift/1000
        # self.dist_per_pixel = linear.steps_to_distance(steps=self.steps_per_pixel,
                                                #    threadpitch=self.motor_worker.xthreadpitch,
                                                #    steps_per_rev=self.motor_worker.__stage.xmotor.steps_per_rev)
        self.motorspeed_change_signal.emit(self.motor_worker.speed_range)


    def run_when_stopped(self, func, *args, **kwargs):
        if self.get_movement() is True:
            return
        elif self.get_movement() is False:
            self.moving_timer.stop()
            logging.info(f"Arrived to position! Function run: {func.__name__}")
            func(*args, **kwargs)

    def objective_change(self):
        self.microns_per_pixel = self.config["objectives"][self.objective_combo.currentText()]["pixel_to_distance"]

# CAMERA RELATED FUNCTIONS
    def start_stop_camera(self):
        if self.camera_start_button.isChecked():
            self.camera_start_signal.emit()
        else:
            self.camera_stop_signal.emit()

    def update_image(self):
        self.imItem.setImage(image=self.camera_worker.camera.image, autoLevels=False, levels=None, lut=False, autoDownsample=False)

    def set_exposure(self):
        value = self.exposure_slider.value()
        disp_string = f"{value} ms"
        self.exposure_display.setText(disp_string)
        # Set camera as well
        self.set_exposure_signal.emit(value*1000)

    def set_gain(self):
        value = self.gain_slider.value()
        disp_string = f"{value}"
        self.gain_display.setText(disp_string)
        # Set camera as well

    def pixel_to_distance(self, pixels):
        return self.microns_per_pixel*pixels
    
    def register_image_to_map(self):
        # Grab an image
        if self.camera_start_button.isChecked():
            image = self.camera_worker.camera.image
            self.fixed_image = pg.ImageItem(image=image)

            newdx = -np.shape(image)[1]/2*self.microns_per_pixel-self.motor_worker.xpos
            newdy = -np.shape(image)[0]/2*self.microns_per_pixel+self.motor_worker.ypos

            tr = QtGui.QTransform()
            tr.translate(newdx,newdy)
            tr.scale(self.microns_per_pixel,self.microns_per_pixel)
            self.fixed_image.setTransform(tr)

            self.image_view_area.addItem(self.fixed_image)

            print(f"Size of image array: {sys.getsizeof(image)/1000000}")
        else:
            logging.error("Camera is not enabled, cannot grab image!")


# OTHER FUNCTIONS
    def load_config(self):
        with open('config.yaml', 'r') as file:
                self.config = yaml.safe_load(file)

    def closeEvent(self, event):
        quit_msg = "Are you sure you want to exit the program?"
        logging.info(quit_msg)
        reply = QtWidgets.QMessageBox.question(self, 'Message', quit_msg, QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No)

        if reply == QtWidgets.QMessageBox.Yes:
            event.accept()
            try:
                self.stage_controller.close()
                logging.info(f'{self.stage_controller.port} port closed')
            except:
                pass
        else:
            event.ignore()

class SingleImage():
    """Class for storing image data from the microscope"""
    def __init__(self, image=None, xloc = 0, yloc = 0):

        self.image = image
        self.xloc = xloc
        self.yloc = yloc

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    ex = MicroscopeApp()
    ex.show()
    sys.exit(app.exec())