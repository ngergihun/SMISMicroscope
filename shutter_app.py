import sys
import yaml

from PySide6.QtCore import QSize, Qt
from PySide6.QtWidgets import (QApplication, QWidget, QMainWindow, 
                               QPushButton, QVBoxLayout, QHBoxLayout, 
                               QButtonGroup, QFrame, QLabel, QCheckBox, 
                               QComboBox, QSpacerItem, QSizePolicy)
from qt_material import apply_stylesheet

from serialStepper import shutter

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

# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Beampath configurator")
        apply_stylesheet(app, theme='light_blue.xml', invert_secondary=True)
        # Main layout
        layout = QHBoxLayout()
        container = QWidget()
        container.setLayout(layout)

        # Container for the control of each servo (populated with the device control widgets later)
        control_layout = QVBoxLayout()
        control_container = QFrame()
        control_container.setLayout(control_layout)

        # Container for the control of each servo (populated with the device control widgets later)
        config_layout = QVBoxLayout()
        config_container = QWidget()
        config_container.setLayout(config_layout)
        # Widgets to the config area
        config_cb_label = QLabel("Select configuration")
        self.config_cb = QComboBox()
        config_button = QPushButton("Set")
        # Widget to the control area
        servo_frame = ShutterControlWidget(test_config_dict)

        config_layout.addWidget(self.config_cb)
        config_layout.addWidget(config_cb_label)
        config_layout.addWidget(config_button)
        config_layout.addSpacerItem(QSpacerItem(20, 40, 
                            QSizePolicy.Policy.Minimum, 
                            QSizePolicy.Policy.Expanding))
        
        control_layout.addWidget(servo_frame)
        control_layout.addSpacerItem(QSpacerItem(20, 40, 
                            QSizePolicy.Policy.Minimum, 
                            QSizePolicy.Policy.Expanding))
        
        layout.addWidget(control_container)
        layout.addWidget(config_container)

        # Set the central widget of the Window.
        self.setCentralWidget(container)
        # -------------------------------------------------------------------------

        # Class attributes
        self.config = None

        self.load_config()

    def add_shutter(self, config: dict):
        pass

    def load_config(self):
        with open('shutter_config.yaml', 'r') as file:
            self.config = yaml.safe_load(file)

class ShutterDevice():
    def __init__(self,ui,device=None):
        self.ui = ui
        self.device = device

class ShutterControlWidget(QFrame):

    
    def __init__(self,config_dict):
        super().__init__()

        layout = QVBoxLayout(self)

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