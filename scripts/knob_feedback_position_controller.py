#!/usr/bin/env python3

from spatialmath import SO3, SE3
import sys
import signal
import rospy
import math
from std_msgs.msg import String, Int32, Float32
from knob_robot_control.msg import KnobState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from franka_msgs.msg import FrankaState
from scipy.spatial.transform import Rotation as R
import numpy as np


class RobotController:

    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        # Handle SIGINT (Ctrl+C)
        signal.signal(signal.SIGINT, self.handle_exit)
        signal.signal(signal.SIGTERM, self.handle_exit) 

        self.publish_rate = rospy.Rate(100)

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

        self.knob_force_command_msg = Float32()
        self.knob_force_command_msg.data = 0.0 
        
        rospy.set_param('reset_pose', False)    
        
        # Publishers and Subscribers
        self.fri_cartesian_move_publisher = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)
        self.knob_state_subscriber = rospy.Subscriber("/knob_state", KnobState, self.knob_state_callback)
        self.franka_state_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
        self.knob_force_pub = rospy.Publisher("/tcp_force", Float32, queue_size=1)

    def handle_exit(self, signum, frame):
        """
        Handle SIGINT (Ctrl+C) for graceful shutdown.
        """
        rospy.loginfo("Ctrl+C detected. Shutting down the Robot Controller...")
        rospy.signal_shutdown("Shutting down due to Ctrl+C")
        sys.exit(0)

    def franka_state_callback(self, data):
    	
        # cartesian TCP wrench
        if self.controlled_axis == 'x':
            self.tcp_contact_force = data.K_F_ext_hat_K[0]
        elif self.controlled_axis == 'y':
            self.tcp_contact_force = data.K_F_ext_hat_K[1]
        elif self.controlled_axis == 'z':
            self.tcp_contact_force = data.K_F_ext_hat_K[2]
           
        # Convert O_T_EE matrix to SE3
        robot_tcp_matrix = np.array(data.O_T_EE).reshape(4,4).T
        
        # Extract and normalize rotation matrix
        R_matrix = robot_tcp_matrix[:3,:3]
        for i in range(3):
            R_matrix[:,i] /= np.linalg.norm(R_matrix[:,i])
        
        so3 = SO3(R_matrix, check=False)
        # Create SE3 from SO3 and set translation
        tcp_pose = SE3(so3)
        tcp_pose.t = robot_tcp_matrix[:3,3]
        
        # Extract position and orientation
        self.robot_tcp_position_current = [
            round(tcp_pose.t[0], 3),
            round(tcp_pose.t[1], 3),
            round(tcp_pose.t[2], 3)
        ]
        self.robot_tcp_orientation_current = so3.rpy(order='xyz')
        
        if not self.ROBOT_STATE_INITIALIZED:
            self.robot_tcp_position_target = self.robot_tcp_position_current.copy()
            self.robot_tcp_orientation_target = self.robot_tcp_orientation_current.copy()
            self.ROBOT_STATE_INITIALIZED = True

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

            #if self.knob_position_delta != 0:
                #print("position current:", self.knob_position_current, "position last:", self.knob_position_last, "position delta:", self.knob_position_delta)
            
        self.knob_position_last = self.knob_position_current

    def check_reset_pose(self):
        if rospy.get_param('reset_pose'):        
            self.robot_tcp_position_target = self.robot_tcp_position_current
            rospy.set_param('reset_pose', False)
            
    def update_robot_command_pose(self):
        
        if self.controlled_axis == 'x':
            self.robot_tcp_position_target[0] -= self.position_change
        elif self.controlled_axis == 'y':
            self.robot_tcp_position_target[1] -= self.position_change
        elif self.controlled_axis == 'z':
            self.robot_tcp_position_target[2] -= self.position_change
        
    
    def publish_knob_force(self):

        tcp_force_feedback_ratio = rospy.get_param("/tcp_force_feedback_ratio")
        self.tcp_force_offset = rospy.get_param("/tcp_force_offset")
        self.knob_command_force = tcp_force_feedback_ratio*(self.tcp_contact_force - self.tcp_force_offset)
        
        if abs(self.knob_command_force) < 0.1:
            self.knob_command_force = 0

        self.CLAMP_FORCE_THRESHOLD_MAX = rospy.get_param('clamp_force_threshold_max')
        self.CLAMP_FORCE_THRESHOLD_MIN = rospy.get_param('clamp_force_threshold_min')

        if self.knob_command_force > 0:
            clamp_force = min(self.CLAMP_FORCE_THRESHOLD_MAX, max(self.CLAMP_FORCE_THRESHOLD_MIN, self.knob_command_force))
        else:
            clamp_force = max(-1*self.CLAMP_FORCE_THRESHOLD_MAX, min(-1*self.CLAMP_FORCE_THRESHOLD_MIN, self.knob_command_force))

        self.knob_force_command_msg.data = float(clamp_force)
        self.knob_force_pub.publish(self.knob_force_command_msg)

    def publish_tcp_cartesian_pose(self):
        self.update_robot_command_pose()
        
        # Create SO3 from RPY angles
        so3 = SO3.RPY(self.robot_tcp_orientation_target, order='xyz')
        # Create SE3 from SO3 and set translation
        target_pose = SE3(so3)
        target_pose.t = self.robot_tcp_position_target
        
        # Update position
        self.pose.pose.position.x = self.robot_tcp_position_target[0]
        self.pose.pose.position.y = self.robot_tcp_position_target[1]
        self.pose.pose.position.z = self.robot_tcp_position_target[2]        

        # Convert to quaternion
        quat = target_pose.UnitQuaternion()
        self.pose.pose.orientation.x = quat.vec3[0]
        self.pose.pose.orientation.y = quat.vec3[1]
        self.pose.pose.orientation.z = quat.vec3[2]
        self.pose.pose.orientation.w = quat.s

        self.fri_cartesian_move_publisher.publish(self.pose)

    def run(self) -> None:
        while not rospy.is_shutdown():
            
            if self.ROBOT_STATE_INITIALIZED:
                self.controlled_axis = rospy.get_param('controlled_axis')
                self.check_reset_pose()
                self.knob_change_update()
                self.publish_tcp_cartesian_pose()
                self.publish_knob_force()
                self.publish_rate.sleep()

if __name__ == '__main__':

    controller = RobotController()
    controller.run()
