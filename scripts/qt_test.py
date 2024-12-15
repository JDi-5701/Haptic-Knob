#!/usr/bin/env python3

import sys
import signal
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QVBoxLayout,
    QWidget,
    QSlider,
    QRadioButton,
    QButtonGroup,
    QPushButton,
)
from PyQt5.QtCore import Qt, QTimer
import rospy
from knob_robot_control.msg import KnobState

class ROSVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.latest_position = 0  # Default position value

        # Initialize ROS node and subscriber
        rospy.init_node('gui_visualizer', anonymous=True)
        rospy.Subscriber('/knob_state', KnobState, self.callback)

        # Initialize ROS parameters
        self.param_position_factor = "position_factor"
        self.param_tcp_force_feedback_ratio = "tcp_force_feedback_ratio"
        self.param_axis = "controlled_axis"
        self.param_reset_pose = "reset_pose"

        # Set initial ROS parameter values
        rospy.set_param(self.param_position_factor, 0.00010)
        rospy.set_param(self.param_tcp_force_feedback_ratio, 0.01)
        rospy.set_param(self.param_axis, "z")
        rospy.set_param(self.param_reset_pose, False)

    def initUI(self):
        """Set up the GUI layout."""
        self.setWindowTitle("ROS Topic Visualizer with Controls")
        
        # Label to display the value from the topic
        self.topic_label = QLabel("Current Knob Position: 0", self)
        self.topic_label.setStyleSheet("font-size: 18px; color: blue;")

        # Slider to adjust the position_factor ROS parameter
        self.position_factor_slider = QSlider(Qt.Horizontal, self)
        self.position_factor_slider.setMinimum(50)   # Representing 0.00010
        self.position_factor_slider.setMaximum(500)   # Representing 0.00050
        self.position_factor_slider.setValue(50)     # Initial value
        self.position_factor_slider.valueChanged.connect(self.updatePositionFactor)

        # Label to display the current position_factor value
        self.position_factor_label = QLabel("Position Factor: 0.00010", self)
        self.position_factor_label.setStyleSheet("font-size: 18px; color: green;")
        
        self.position_factor_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid #bbb;
                background: #eee;
                height: 8px;
                border-radius: 4px;
            }

            QSlider::handle:horizontal {
                background: #388E3C;  /* Green */
                border: 1px solid #5c5c5c;
                width: 18px;
                margin: -7px 0; /* Center handle */
                border-radius: 9px;
            }

            QSlider::handle:horizontal:hover {
                background: #4CAF50; /* Lighter green when hovered */
            }

            QSlider::sub-page:horizontal {
                background: #76FF03; /* Lighter green to show progress */
                border: 1px solid #777;
                height: 8px;
                border-radius: 4px;
            }

            QSlider::add-page:horizontal {
                background: #ccc;
                border: 1px solid #aaa;
                height: 8px;
                border-radius: 4px;
            }
        """)

        # Slider to adjust the tcp_force_feedback_ratio ROS parameter
        self.tcp_force_slider = QSlider(Qt.Horizontal, self)
        self.tcp_force_slider.setMinimum(1)   # Representing 0.01
        self.tcp_force_slider.setMaximum(20)  # Representing 0.2
        self.tcp_force_slider.setValue(1)     # Initial value
        self.tcp_force_slider.valueChanged.connect(self.updateTcpForceFeedback)

        # Label to display the current tcp_force_feedback_ratio value
        self.tcp_force_label = QLabel("TCP Force Feedback Ratio: 0.01", self)
        self.tcp_force_label.setStyleSheet("font-size: 18px; color: red;")
        
        self.tcp_force_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid #bbb;
                background: #eee;
                height: 8px;
                border-radius: 4px;
            }

            QSlider::handle:horizontal {
                background: #cf310f;  /* Red */
                border: 1px solid #5c5c5c;
                width: 18px;
                margin: -7px 0; /* Center handle */
                border-radius: 9px;
            }

            QSlider::handle:horizontal:hover {
                background: #f04d2a; /* Lighter red when hovered */
            }

            QSlider::sub-page:horizontal {
                background: #f04d2a; /* Lighter green to show progress */
                border: 1px solid #777;
                height: 8px;
                border-radius: 4px;
            }

            QSlider::add-page:horizontal {
                background: #ccc;
                border: 1px solid #aaa;
                height: 8px;
                border-radius: 4px;
            }
        """)

        # Radio buttons for controlled_axis
        self.radio_x = QRadioButton("X")
        self.radio_y = QRadioButton("Y")
        self.radio_z = QRadioButton("Z")
        self.radio_z.setChecked(True)  # Default selection

        # Group the radio buttons
        self.axis_group = QButtonGroup(self)
        self.axis_group.addButton(self.radio_x)
        self.axis_group.addButton(self.radio_y)
        self.axis_group.addButton(self.radio_z)

        # Connect the radio buttons to the callback
        self.radio_x.toggled.connect(self.updateAxis)
        self.radio_y.toggled.connect(self.updateAxis)
        self.radio_z.toggled.connect(self.updateAxis)

        # Label to show the selected axis
        self.axis_label = QLabel("Controlled Axis: X", self)
        self.axis_label.setStyleSheet("font-size: 18px; color: purple;")

        # Button to reset target position
        self.reset_button = QPushButton("Reset Target Position", self)
        self.reset_button.clicked.connect(self.resetTargetPosition)

        # Set layout
        layout = QVBoxLayout()
        layout.addWidget(self.topic_label)
        layout.addWidget(self.position_factor_slider)
        layout.addWidget(self.position_factor_label)
        layout.addWidget(self.tcp_force_slider)
        layout.addWidget(self.tcp_force_label)
        layout.addWidget(self.radio_x)
        layout.addWidget(self.radio_y)
        layout.addWidget(self.radio_z)
        layout.addWidget(self.axis_label)
        layout.addWidget(self.reset_button)
        self.setLayout(layout)
        
        # Timer to periodically refresh the topic value
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateLabel)
        self.timer.start(100)  # Refresh every 100 ms

    def callback(self, msg):
        """Callback to receive data from the ROS topic."""
        self.latest_position = msg.position  # Extract position value from the message

    def updateLabel(self):
        """Update the label with the latest topic data."""
        self.topic_label.setText(f"Current Position: {self.latest_position}")

    def updatePositionFactor(self):
        """Update the position_factor ROS parameter based on the slider's value."""
        param_value = self.position_factor_slider.value() / 500000.0  # Convert to float (0.00010 to 0.00050)
        rospy.set_param(self.param_position_factor, param_value)
        self.position_factor_label.setText(f"Position Factor: {param_value:.5f}")

    def updateTcpForceFeedback(self):
        """Update the tcp_force_feedback_ratio ROS parameter based on the slider's value."""
        param_value = self.tcp_force_slider.value() / 100.0  # Convert to float (0.01 to 0.2)
        rospy.set_param(self.param_tcp_force_feedback_ratio, param_value)
        self.tcp_force_label.setText(f"TCP Force Feedback Ratio: {param_value:.2f}")

    def updateAxis(self):
        """Update the ROS parameter controlled_axis based on the selected radio button."""
        if self.radio_x.isChecked():
            axis = "x"
        elif self.radio_y.isChecked():
            axis = "y"
        elif self.radio_z.isChecked():
            axis = "z"
        else:
            return
        rospy.set_param(self.param_axis, axis)
        self.axis_label.setText(f"Controlled Axis: {axis.upper()}")

    def resetTargetPosition(self):
        """Set reset_pose to True for 3 seconds."""
        rospy.set_param(self.param_reset_pose, True)
        QTimer.singleShot(3000, self.clearResetPose)  # Reset after 3 seconds

    def clearResetPose(self):
        """Set reset_pose back to False."""
        rospy.set_param(self.param_reset_pose, False)

def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully shut down the GUI."""
    print("Ctrl+C detected. Shutting down GUI...")
    QApplication.quit()

if __name__ == "__main__":
    # Handle Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Create a Qt application
    app = QApplication(sys.argv)

    # Create and show the GUI
    window = ROSVisualizer()
    window.show()

    # Run the Qt application while letting rospy spin
    try:
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
