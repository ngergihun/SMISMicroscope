import sys
import yaml

from PySide6.QtCore import QSize, Qt, QObject, QThread, Signal, Slot, QTimer
from PySide6.QtWidgets import (QApplication, QWidget, QMainWindow, 
                               QPushButton, QVBoxLayout, QHBoxLayout, 
                               QButtonGroup, QFrame, QLabel, QCheckBox, 
                               QComboBox, QSpacerItem, QSizePolicy)
from qt_material import apply_stylesheet

from serialStepper import shutter, Controller

test_config_dict = {
    "name": "SimpleShutter",
    "pos_A_name": "Power Meter",
    "pos_B_name": "Input",
    "servo_A": 120,
    "servo_B": 280,
    "servo_tol": 5,
    "sensor_open": 1,
    "servo_open": 1,
    "adress_servo": str(0),
    "address_sensor": str(1),
}

class WorkerThread(QObject):
    """Qt thread handling the servo control independently from the UI"""
    pos_updated = Signal(tuple)

    def __init__(self,config,debug=False):
        super().__init__()

        self.config = config
        self.servo_dict = {}

        self.moving_timer = QTimer()
        self.moving_timer.setTimerType(Qt.PreciseTimer)

        self.controller = Controller(port=self.config["port"], debug=debug)

    def create_shutters(self):
        for s in self.config["shutters"]:
            self.add_shutter(s)
    
    @Slot()
    def add_shutter(self,name):
        if name not in list(self.servo_dict.keys()):
            print(self.config["shutters"][name])
            self.servo_dict[name] = shutter.ShutterWithSensor(controller=self.controller,
                                                              config=self.config["shutters"][name],
                                                              debug=False) #self.config["shutters"][name]# ShutterWithSensor()
            print(f"Creating the servo object {list(self.servo_dict.keys())[-1]}")
        else:
            raise ValueError
    @Slot()
    def remove_shutter(self,name):
        print(f"Deleting the servo object {name}")
        del self.servo_dict[name]

    def update_positions(self,shutter):
        for s in self.servo_dict:
            self.servo_dict[s].get_state()
            self.pos_updated.emit((s,bool(shutter.switch.state.value)))

# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):

    addShutter = Signal(str)
    deleteShutter = Signal(str)

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Beampath configurator")
        apply_stylesheet(app, theme='light_blue.xml', invert_secondary=True)
        # Main layout
        layout = QHBoxLayout()
        container = QWidget()
        container.setLayout(layout)

        # Container for the control of each servo (populated with the device control widgets later)
        self.control_layout = QVBoxLayout()
        control_container = QFrame()
        control_container.setLayout(self.control_layout)

        # Container for the control of each servo (populated with the device control widgets later)
        config_layout = QVBoxLayout()
        config_container = QWidget()
        config_container.setLayout(config_layout)
        # Widgets to the config area
        config_cb_label = QLabel("Select configuration")
        self.config_cb = QComboBox()
        config_button = QPushButton("Set")
        # Widget to the control area
        config_layout.addWidget(self.config_cb)
        config_layout.addWidget(config_cb_label)
        config_layout.addWidget(config_button)
        config_layout.addSpacerItem(QSpacerItem(20, 40, 
                            QSizePolicy.Policy.Minimum, 
                            QSizePolicy.Policy.Expanding))
        
        self.control_layout.addSpacerItem(QSpacerItem(20, 40, 
                            QSizePolicy.Policy.Minimum, 
                            QSizePolicy.Policy.Expanding))
        
        layout.addWidget(control_container)
        layout.addWidget(config_container)

        # Set the central widget of the Window.
        self.setCentralWidget(container)
        # -------------------------------------------------------------------------
        # Class attributes
        self.config = None
        self.shutter_ui_dict = {}
        self.load_config()

        self.worker = WorkerThread(config=self.config,debug=False)
        self.worker_thread = QThread()
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.start()

        # Connect signals:
        self.addShutter.connect(self.worker.add_shutter)
        self.deleteShutter.connect(self.worker.remove_shutter)
        config_button.clicked.connect(self.worker.update_positions)

        self.add_all_shutters()

        self.worker.pos_updated.connect(self.on_sensor_update)

    def add_shutter(self, name: str, single_config: dict):
        self.shutter_ui_dict[name] = ShutterControlWidget(single_config,name)
        self.control_layout.addWidget(self.shutter_ui_dict[name])
        self.addShutter.emit(name)
        
    def remove_shutter(self,name):
        self.control_layout.removeWidget(self.shutter_ui_dict[name])
        self.shutter_ui_dict[name].deleteLater()
        self.deleteShutter.emit(name)
        
    def add_all_shutters(self):
        for s in list(self.config["shutters"].keys()):
            self.add_shutter(name=s,single_config=self.config["shutters"][s])

    def remove_all_shutter(self):
        for s in list(self.config["shutters"].keys()):
            self.remove_shutter(name=s)

    def load_config(self):
        with open('shutter_config.yaml', 'r') as file:
            self.config = yaml.safe_load(file)

    def on_sensor_update(self,state):
        print(state)

class ShutterControlWidget(QFrame):

    def __init__(self,config_dict,name):
        super().__init__()

        layout = QVBoxLayout(self)
        
        self.name = name # reference name for the backend, not the same as the displayed name
        self.name_label = QLabel(f'Shutter: {config_dict["name"]}')
        # Create two toggleable buttons
        self.btn_moveA = QPushButton(config_dict["pos_A_name"])
        self.btn_moveA.setCheckable(True)
        self.btn_moveB = QPushButton(config_dict["pos_B_name"])
        self.btn_moveB.setCheckable(True)

        button_container_layout = QHBoxLayout()
        button_container = QWidget()
        button_container.setLayout(button_container_layout)
        # Put them in a button group
        group = QButtonGroup(self)
        group.setExclusive(True)        # ensures only one can be checked at a time
        group.addButton(self.btn_moveA)
        group.addButton(self.btn_moveB)

        # Add a checkbox to display the status
        checkbox = QCheckBox('Status')
        checkbox.setCheckable(False)
        checkbox.setLayoutDirection(Qt.RightToLeft)

        # Optional: connect to signal to see which one is selected
        group.buttonToggled.connect(self.__on_toggled)

        layout.addWidget(self.name_label)
        layout.addWidget(button_container)
        button_container_layout.addWidget(self.btn_moveA)
        button_container_layout.addWidget(self.btn_moveB)
        button_container_layout.addWidget(checkbox)
    
    def __on_toggled(self, button, checked):
        if checked:
            print(f"{button.text()} is now active")

if __name__ == '__main__':
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    app.exec()