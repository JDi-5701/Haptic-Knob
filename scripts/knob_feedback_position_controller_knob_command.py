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
        self.knob_command_pub = rospy.Publisher("/knob_command", KnobCommand, queue_size=1)
        #TODO: publish robot tcp frame and joint from FRI
        #self.iiwa_driver_joint_states = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        #self.robot_tcp_state_subscriber = rospy.Subscriber("/tool_frame", EulerFrame, self.tool_frame_subscribe_callback)

        self.robot_tcp_position_current = [-0.0000, 0.662, 0.330]
        self.robot_tcp_orientation_current = [0.0, -0.0, 3.14]
        self.robot_tcp_force_current = [0, 0, 0]

        self.robot_tcp_position_target = [-0.0000, 0.662, 0.330]
        self.robot_tcp_orientation_target = [0.0, -0.0, 3.14]
        self.robot_tcp_force_threshold = [5, 5, 5]

        self.knob_force = 0
        self.knob_position = 0
        self.knob_position_delta = 0
        self.knob_command_force = 0

        self.force_factor = 1
        self.velocity_factor = 1

        self.velocity = 0.001

        self.VELOCITY_THRESHOLD = 0.03

        #self.joints = [1.5710205516437281, 0.26206090357094514, -2.6964464278686393e-05, -1.2529596421209066, 7.200128936299848e-05, 1.6281237054938813, -1.570994186372798]

        self.ROBOT_STATE_INITIALIZED = False

        rospy.set_param("/speed_factor", 0.00001)
        self.SPEED_FACTOR = rospy.get_param("/speed_factor")

        rospy.set_param("/position_factor", 0.00005)
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

        self.BOUNDING_MODE = False

        self.tcp_pose_error = 0.0

        self.knob_command = KnobCommand()

        rospy.set_param("/force_pose_error_ratio", 0.000001)
        rospy.set_param("/tcp_force_feedback_ratio", 0.07)

        rospy.set_param('clamp_force_threshold_max', 2)
        self.CLAMP_FORCE_THRESHOLD_MAX = rospy.get_param('clamp_force_threshold_max')
        rospy.set_param('clamp_force_threshold_min', 0)
        self.CLAMP_FORCE_THRESHOLD_MIN = rospy.get_param('clamp_force_threshold_min')

        rospy.set_param('tcp_force_offset', 19)
        self.tcp_force_offset = 19

        self.knob_commanded_force = 0.0

        self.tcp_contact_force = 0.0

        self.knob_command_bound_force = 0.0

        self.knob_position_current = 0
        self.knob_position_delta = 0
        self.knob_position_last = 0        

    def tcp_wrench_callback(self, data) -> None:    
        self.tcp_contact_force = data.wrench.force.z

    def knob_state_callback(self, data):
        self.knob_position_current = data.position.data
        self.knob_force = data.force.data

        self.knob_position_delta = self.knob_position_current - self.knob_position_last

        if self.knob_position_delta > 100:
            self.knob_position_delta = 0
            self.knob_position_last = self.knob_position_current
        
        self.position_factor = rospy.get_param("/position_factor")
        self.position_change = self.position_factor * self.knob_position_delta
        self.update_robot_command_pose()

        if self.knob_position_delta != 0:
            print("position current:", self.knob_position_current, "position last:", self.knob_position_last, "position delta:", self.knob_position_delta)
        
        self.knob_position_last = self.knob_position_current

    def update_robot_command_pose(self):
        self.robot_tcp_position_target[2] -= self.position_change

    def robot_cartesian_pose_callback(self, data):
        pose_position = data.pose.position
        pose_orientation = data.pose.orientation
        self.robot_tcp_position_current = [round(pose_position.x, 3), round(pose_position.y, 3), round(pose_position.z, 3)]
        self.robot_tcp_orientation_current = [round(pose_orientation.x, 3), round(pose_orientation.y,3), round(pose_orientation.z,3)]
        self.detect_tcp_axis_block()
    
    def detect_tcp_axis_block(self):
        self.tcp_pose_error = math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(self.robot_tcp_position_current, self.robot_tcp_position_target)))
        #print("tcp pose error", self.tcp_pose_error, "knob force tcp error part", round(self.knob_command_bound_force, 3))
        
        if self.tcp_pose_error > 0.002: 
            self.BOUNDING_MODE = True
            self.update_bound_force()
        else:
            self.BOUNDING_MODE = False
            self.knob_command_bound_force = 0

    def update_bound_force(self):

        force_pose_error_ratio = rospy.get_param("/force_pose_error_ratio")
        self.knob_command_bound_force = abs(self.tcp_pose_error * force_pose_error_ratio)      
    
    def publish_knob_force(self):

        tcp_force_feedback_ratio = rospy.get_param("/tcp_force_feedback_ratio")
        self.tcp_force_offset = rospy.get_param("/tcp_force_offset")
        self.knob_command_force = abs(tcp_force_feedback_ratio*(self.tcp_contact_force - self.tcp_force_offset)) + self.knob_command_bound_force
        
        #print("knob force: {}.\n".format(self.knob_command_force))

        if self.knob_command_force < 0.1:
            self.knob_command_force = 0

        self.CLAMP_FORCE_THRESHOLD_MAX = rospy.get_param('clamp_force_threshold_max')
        self.CLAMP_FORCE_THRESHOLD_MIN = rospy.get_param('clamp_force_threshold_min')
        #clamp force lies in between 2 thresholds
        clamp_force = min(self.CLAMP_FORCE_THRESHOLD_MAX, max(self.CLAMP_FORCE_THRESHOLD_MIN, self.knob_command_force))
        #knob_command.detent_strength_unit.data = detent_strength_unit
        self.knob_command.text.data = "force"
        self.knob_command.tcp_force.data = float(clamp_force)
        self.knob_command_pub.publish(self.knob_command)


    def publish_tcp_cartesian_pose(self) -> bool:
        self.pose.pose.position.z = self.robot_tcp_position_target[2]

        # Publish the Cartesian pose
        self.fri_cartesian_move_publisher.publish(self.pose)
        #print("Sending fri position to robot: {}.\n".format(self.pose.pose.position.z))
        
        return True

    def velocity_calculator(self):
        error = self.robot_tcp_position_target[2] - self.robot_tcp_position_current[2]
        self.SPEED_FACTOR = rospy.get_param("/speed_factor")
        self.velocity = error * self.SPEED_FACTOR + 0.01

        if self.velocity > self.VELOCITY_THRESHOLD:
            self.velocity = self.VELOCITY_THRESHOLD

    def run(self) -> None:
        while not rospy.is_shutdown():
                       
            #self.velocity_calculator()

            self.publish_tcp_cartesian_pose()
            self.publish_knob_force()
            self.publish_rate.sleep()

if __name__ == '__main__':

    controller = RobotController()
    controller.run()

