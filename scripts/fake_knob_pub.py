#!/usr/bin/env python3

import rospy
from knob_robot_control.msg import KnobState

rospy.init_node('fake_state_pub')
pub = rospy.Publisher('/knob_state', KnobState, queue_size=10)

rate = rospy.Rate(10) # 10hz
import math

while not rospy.is_shutdown():
    fake_state = KnobState()
    fake_state.header.stamp = rospy.Time.now()
    fake_state.mode.data = "TCP"
    fake_state.position.data = int(100 * math.sin(rospy.Time.now().to_sec()))
    fake_state.force.data = 0
    
    pub.publish(fake_state)
    rate.sleep()
