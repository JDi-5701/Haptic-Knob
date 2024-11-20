#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-09-10
import sys
import math
import rospy
from knob_robot_control.msg import KnobState, KnobCommand
import argparse

class Knob_Setup():
    def __init__(self):
        rospy.init_node('knob_publisher')  # Initialize ROS node
        self.knob_command_pub = rospy.Publisher("/knob_command", KnobCommand, queue_size=10)

    def publish_knob_command(self, position, num_positions, position_width_radians, detent_strength_unit) -> None:  
        knob_command = KnobCommand()
        knob_command.header.stamp = rospy.Time.now()
        knob_command.num_positions.data = int(num_positions)
        knob_command.position.data = int(position)
        knob_command.position_width_radians.data = float(position_width_radians)
        knob_command.detent_strength_unit.data = float(detent_strength_unit)
        knob_command.endstop_strength_unit.data = float(1.0)
        knob_command.snap_point.data = float(1.1)
        knob_command.text.data = "manual"
        knob_command.tcp_force.data = float(1.0)    
        self.knob_command_pub.publish(knob_command)

if __name__ == "__main__":

    knob_setup = Knob_Setup()

    # Rate at which to publish messages (adjust as needed)
    rate = rospy.Rate(50)  # 50 Hz

    while not rospy.is_shutdown():

        num_positions = rospy.get_param('num_positions')
        position = rospy.get_param('position')
        position_width_radians = rospy.get_param('position_width_radians')
        detent_strength_unit = rospy.get_param('detent_strength_unit')

        # Publish message
        knob_setup.publish_knob_command(num_positions, position, position_width_radians, detent_strength_unit)
        
        # Sleep to maintain desired rate
        rate.sleep()
