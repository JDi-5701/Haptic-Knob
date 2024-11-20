#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-09-01
# Description: This script is used to control the robot arm through the robot movement interface.

import rospy
import math
#from robot_movement_interface.msg import Command, CommandList, EulerFrame
#from robot_movement_interface.msg import Result
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Float32
from knob_robot_control.msg import KnobState, KnobCommand
from geometry_msgs.msg import WrenchStamped, PoseStamped
from threading import Lock


class RobotController:

    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        self.publish_rate = rospy.Rate(100)
        
        #Publishers and Subscribers
        self.fri_cartesian_move_publisher = rospy.Publisher('/fri_cartesian_command', PoseStamped, queue_size=10)
        self.knob_state_subscriber = rospy.Subscriber("/knob_state", KnobState, self.knob_state_callback)
        self.robot_tcp_cartesian_subscriber = rospy.Subscriber("/fri_cartesian_pose", PoseStamped, self.robot_cartesian_pose_callback)
        self.tcp_wrench_sub = rospy.Subscriber("/ft_sensor/netft_data", WrenchStamped, self.tcp_wrench_callback)
        self.knob_force_pub = rospy.Publisher("/tcp_force", Float32, queue_size=1)

        self.robot_tcp_position_current = [-0.0000, 0.662, 0.330]
        self.robot_tcp_orientation_current = [0.0, -0.0, 3.14]
        self.robot_tcp_force_current = [0, 0, 0]

        self.robot_tcp_position_target = [-0.0000, 0.662, 0.330]
        self.robot_tcp_orientation_target = [0.0, -0.0, 3.14]

        self.knob_force = 0
        self.knob_position = 0
        self.knob_position_delta = 0
        self.knob_command_force = 0

        self.ROBOT_STATE_INITIALIZED = False

        rospy.set_param("/position_factor", 0.00003)
        self.position_factor = rospy.get_param("/position_factor")
        
        self.position_change = 0.0

        # Create a PoseStamped message
        self.pose = PoseStamped()
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = 'base_link'
        self.pose.pose.position.x = self.robot_tcp_position_target[0]
        self.pose.pose.position.y = self.robot_tcp_position_target[1]
        self.pose.pose.position.z = self.robot_tcp_position_target[2]
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 1.0

        self.knob_command = KnobCommand()

        rospy.set_param("/tcp_force_feedback_ratio", 0.08)

        rospy.set_param('clamp_force_threshold_max', 2)
        self.CLAMP_FORCE_THRESHOLD_MAX = rospy.get_param('clamp_force_threshold_max')
        rospy.set_param('clamp_force_threshold_min', 0)
        self.CLAMP_FORCE_THRESHOLD_MIN = rospy.get_param('clamp_force_threshold_min')

        rospy.set_param('tcp_force_offset', 0)
        self.tcp_force_offset = 0

        rospy.set_param('controlled_axis', 'z')
        self.controlled_axis = 'z'

        self.tcp_contact_force = 0.0

        self.knob_position_current = 0
        self.knob_position_delta = 0
        self.knob_position_last = 0

        # Create a Float32 message object
        self.knob_force_command_msg = Float32()

        # Assign a value to the message
        self.knob_force_command_msg.data = 0.0 
        
        rospy.set_param('reset_pose', False)    

    def tcp_wrench_callback(self, data) -> None:
        if self.controlled_axis == 'x':
            self.tcp_contact_force = data.wrench.force.x
        elif self.controlled_axis == 'y':
            self.tcp_contact_force = data.wrench.force.y
        elif self.controlled_axis == 'z':
            self.tcp_contact_force = data.wrench.force.z

    def knob_state_callback(self, data):
        self.knob_position_current = data.position.data
        self.knob_force = data.force.data
    
    def knob_change_update(self):
        self.knob_position_delta = self.knob_position_current - self.knob_position_last

        if abs(self.knob_position_delta) > 30:
            self.knob_position_delta = 0
        else:
            self.position_factor = rospy.get_param("/position_factor")
            self.position_change = self.position_factor * self.knob_position_delta

            if self.knob_position_delta != 0:
                print("position current:", self.knob_position_current, "position last:", self.knob_position_last, "position delta:", self.knob_position_delta)
            
        self.knob_position_last = self.knob_position_current

    def check_reset_pose(self):
        if rospy.get_param('reset_pose'):        
            self.robot_tcp_position_target = self.robot_tcp_position_current
            
    def update_robot_command_pose(self):
        
        if self.controlled_axis == 'x':
            self.robot_tcp_position_target[0] -= self.position_change
        elif self.controlled_axis == 'y':
            self.robot_tcp_position_target[1] -= self.position_change
        elif self.controlled_axis == 'z':
            self.robot_tcp_position_target[2] -= self.position_change
        
    def robot_cartesian_pose_callback(self, data):
        pose_position = data.pose.position
        pose_orientation = data.pose.orientation
        self.robot_tcp_position_current = [round(pose_position.x, 3), round(pose_position.y, 3), round(pose_position.z, 3)]
        self.robot_tcp_orientation_current = [round(pose_orientation.x, 3), round(pose_orientation.y,3), round(pose_orientation.z,3)]
    
    def publish_knob_force(self):

        tcp_force_feedback_ratio = rospy.get_param("/tcp_force_feedback_ratio")
        self.tcp_force_offset = rospy.get_param("/tcp_force_offset")
        self.knob_command_force = tcp_force_feedback_ratio*(self.tcp_contact_force - self.tcp_force_offset)
        
        #print("knob force: {}.\n".format(self.knob_command_force))

        if abs(self.knob_command_force) < 0.1:
            self.knob_command_force = 0

        self.CLAMP_FORCE_THRESHOLD_MAX = rospy.get_param('clamp_force_threshold_max')
        self.CLAMP_FORCE_THRESHOLD_MIN = rospy.get_param('clamp_force_threshold_min')
        #clamp force lies in between 2 thresholds
        if self.knob_command_force > 0:
            clamp_force = min(self.CLAMP_FORCE_THRESHOLD_MAX, max(self.CLAMP_FORCE_THRESHOLD_MIN, self.knob_command_force))
        else:
            clamp_force = max(-1*self.CLAMP_FORCE_THRESHOLD_MAX, min(-1*self.CLAMP_FORCE_THRESHOLD_MIN, self.knob_command_force))

        self.knob_force_command_msg.data = float(clamp_force)
        self.knob_force_pub.publish(self.knob_force_command_msg)

    def publish_tcp_cartesian_pose(self):
        
        self.update_robot_command_pose()
        
        self.pose.pose.position.x = self.robot_tcp_position_target[0]
        self.pose.pose.position.y = self.robot_tcp_position_target[1]
        self.pose.pose.position.z = self.robot_tcp_position_target[2]

        # Publish the Cartesian pose
        self.fri_cartesian_move_publisher.publish(self.pose)
        #print("Sending fri position to robot: {}.\n".format(self.pose.pose.position.z))

    def run(self) -> None:
        while not rospy.is_shutdown():
            
            if not self.ROBOT_STATE_INITIALIZED:
                self.robot_tcp_position_target = self.robot_tcp_position_current
                self.ROBOT_STATE_INITIALIZED = True
                                 
            self.controlled_axis = rospy.get_param('controlled_axis')
            self.check_reset_pose()
            self.knob_change_update()
            self.publish_tcp_cartesian_pose()
            self.publish_knob_force()
            self.publish_rate.sleep()

if __name__ == '__main__':

    controller = RobotController()
    controller.run()

