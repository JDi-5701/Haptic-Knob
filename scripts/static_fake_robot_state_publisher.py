#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from knob_robot_control.msg import KnobState, KnobCommand
import random
import math

class FakePublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('fake_publisher', anonymous=True)

        # Publishers
        self.tcp_wrench_pub = rospy.Publisher('/tcp_wrench', WrenchStamped, queue_size=10)
        self.tool_frame_pub = rospy.Publisher('/tool_frame', PoseStamped, queue_size=10)

        rospy.set_param('fake_force_z', 0.2)

        # Publishing rate
        self.rate = rospy.Rate(50) # 10hz

        self.start_time = rospy.get_time()

        self.position_home = [-0.08374, 0.4547, 0.25418]

        self.wrench_msg = WrenchStamped()
        self.wrench_msg.wrench.force.x = 0.2
        self.wrench_msg.wrench.force.y = 0.2
        self.wrench_msg.wrench.force.z = 0.2
        self.wrench_msg.wrench.torque.x = 0.2
        self.wrench_msg.wrench.torque.y = 0.2
        self.wrench_msg.wrench.torque.z = 0.2

        self.pose_msg = PoseStamped()
        self.pose_msg.pose.position.x = self.position_home[0]
        self.pose_msg.pose.position.y = self.position_home[1]
        self.pose_msg.pose.position.z = self.position_home[2]

        self.pose_msg.pose.orientation.x = math.sin(3.14 / 2)
        self.pose_msg.pose.orientation.y = 0
        self.pose_msg.pose.orientation.z = 0
        self.pose_msg.pose.orientation.w = math.cos(3.14 / 2)

    def publish_tcp_wrench(self):
        current_time = rospy.get_time() - self.start_time
        self.wrench_msg.header.stamp = rospy.Time.now()

        self.wrench_msg.wrench.force.z = rospy.get_param("fake_force_z")
        self.tcp_wrench_pub.publish(self.wrench_msg)

    def publish_tool_frame(self):
        current_time = rospy.get_time() - self.start_time
        self.pose_msg.header.stamp = rospy.Time.now()

        self.tool_frame_pub.publish(self.pose_msg)

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
        fake_publisher = FakePublisher()
        rospy.loginfo("Fake robot state publisher started")
        fake_publisher.run()
    except rospy.ROSInterruptException:
        pass
