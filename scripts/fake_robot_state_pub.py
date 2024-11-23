#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
import random
import math

class FakeRobotStatePublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('fake_publisher', anonymous=True)

        self.knob_current_pos = None
        self.knob_current_force = None

        # Publishers
        self.tcp_wrench_pubpublisher = rospy.Publisher('/tcp_wrench', WrenchStamped, queue_size=10)
        self.tcp_pose_publisher = rospy.Publisher('/fri_cartesian_pose', PoseStamped, queue_size=10)

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

        self.tcp_wrench_pubpublisher.publish(wrench_msg)

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
