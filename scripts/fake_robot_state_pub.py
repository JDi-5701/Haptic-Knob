#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from franka_msgs.msg import FrankaState
import random
import math
from scipy.spatial.transform import Rotation as R
import numpy as np

class FakeRobotStatePublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('fake_publisher', anonymous=True)

        self.knob_current_pos = None
        self.knob_current_force = None

        # Publishers
        self.tcp_wrench_publisher = rospy.Publisher('/tcp_wrench', WrenchStamped, queue_size=10)
        self.tcp_pose_publisher = rospy.Publisher('/fri_cartesian_pose', PoseStamped, queue_size=10)
        
        self.fake_franka_state_publisher = rospy.Publisher('/franka_state_controller/franka_states', FrankaState, queue_size=10)

        # Publishing rate
        self.rate = rospy.Rate(10) # 10hz

        self.start_time = rospy.get_time()

        self.position_home = [-0.08374, 0.4547, 0.25418]

        rospy.set_param("/fake_tcp_force", 0)

    def publish_tcp_wrench(self):
        current_time = rospy.get_time() - self.start_time
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()

        desired_fake_force = rospy.get_param("/fake_tcp_force", 0)

        wrench_msg.wrench.force.x = random.uniform(-1, 1)
        wrench_msg.wrench.force.y = random.uniform(-1, 1)
        wrench_msg.wrench.force.z = desired_fake_force + random.uniform(-1, 1)

        wrench_msg.wrench.torque.x = random.uniform(-1, 1)
        wrench_msg.wrench.torque.y = random.uniform(-1, 1)
        wrench_msg.wrench.torque.z = random.uniform(-1, 1)

        self.tcp_wrench_publisher.publish(wrench_msg)

    def publish_tool_frame(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()

        pose_msg.pose.position.x = self.position_home[0] + random.uniform(-0.1, 0.1)
        pose_msg.pose.position.y = self.position_home[1] + random.uniform(-0.1, 0.1)
        pose_msg.pose.position.z = self.position_home[2] + random.uniform(-0.1, 0.1)
    
        angle = 3.14

        pose_msg.pose.orientation.x = math.sin(angle / 2)
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = math.cos(angle / 2)

        self.tcp_pose_publisher.publish(pose_msg)
    
    def publish_tool_frame(self):
        franka_state_msg = FrankaState()
        franka_state_msg.header.stamp = rospy.Time.now()

        position_x = self.position_home[0] + random.uniform(-0.1, 0.1)
        position_y = self.position_home[1] + random.uniform(-0.1, 0.1)
        position_z = self.position_home[2] + random.uniform(-0.1, 0.1)
    
        angle = 3.14
        
        orientation_x = math.sin(angle / 2)
        orientation_y = 0
        orientation_z = 0
        orientation_w = math.cos(angle / 2)
        
        rotation_matrix = R.from_quat([orientation_x, orientation_y, orientation_z, orientation_w]).as_matrix()
        
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = [position_x, position_y, position_z]
        
        franka_state_msg.O_T_EE = transformation_matrix.flatten().tolist()#here the 1x16 array for pose
        
        desired_fake_force = rospy.get_param("/fake_tcp_force", 0)

        franka_state_msg.K_F_ext_hat_K = [random.uniform(-1, 1), random.uniform(-1, 1), desired_fake_force + random.uniform(-1, 1), 
        					random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)]


        self.fake_franka_state_publisher.publish(franka_state_msg)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.publish_tcp_wrench()
                self.publish_tool_frame()
            except rospy.ROSInterruptException:
                pass
            self.rate.sleep()

if __name__ == '__main__':
    try:
        fake_publisher = FakeRobotStatePublisher()
        rospy.loginfo("Fake robot state publisher started")
        fake_publisher.run()
    except rospy.ROSInterruptException:
        pass
