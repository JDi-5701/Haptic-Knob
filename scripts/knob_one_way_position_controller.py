#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-09-01
# Description: This script is used to control the robot arm through the robot movement interface.

import rospy
#from robot_movement_interface.msg import Command, CommandList, EulerFrame
#from robot_movement_interface.msg import Result
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Float32
from knob_robot_control.msg import KnobState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from threading import Lock


class RobotController:

    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        self.publish_rate = rospy.Rate(200)
        
        #Publishers and Subscribers
        self.fri_cartesian_command_publisher = rospy.Publisher('/fri_cartesian_command', PoseStamped, queue_size=10)
        self.knob_state_subscriber = rospy.Subscriber("/knob_state", KnobState, self.knob_state_callback)
        self.robot_tcp_cartesian_subscriber = rospy.Subscriber("/fri_cartesian_pose", PoseStamped, self.robot_cartesian_callback)
        #TODO: publish robot tcp frame and joint from FRI
        #self.iiwa_driver_joint_states = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        #self.robot_tcp_state_subscriber = rospy.Subscriber("/tool_frame", EulerFrame, self.tool_frame_subscribe_callback)

        self.robot_tcp_position_current = [-0.0000, 0.444, 0.444]
        self.robot_tcp_orientation_current = [0.0, -0.0, 3.14]
        self.robot_tcp_force_current = [0, 0, 0]

        self.robot_tcp_position_target = [-0.0000, 0.444, 0.444]
        self.robot_tcp_orientation_target = [0.0, -0.0, 3.14]
        self.robot_tcp_force_threshold = [5, 5, 5]

        self.knob_force = 0
        self.knob_position = 0
        self.knob_position_delta = 0

        self.position_factor = 0.0001
        self.force_factor = 1
        self.velocity_factor = 1

        self.velocity = 0.001

        self.VELOCITY_THRESHOLD = 0.03

        #self.joints = [1.5710205516437281, 0.26206090357094514, -2.6964464278686393e-05, -1.2529596421209066, 7.200128936299848e-05, 1.6281237054938813, -1.570994186372798]

        self.ROBOT_STATE_INITIALIZED = False
        self.CHANGE_FLAG = False
        self.POSITION_CHANGED = False

        rospy.set_param("/speed_factor", 0.001)
        self.SPEED_FACTOR = rospy.get_param("/speed_factor")
        
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

        self.BOUNGDING_MODE = False

    def knob_state_callback(self, data):
        #self.knob_position_delta = data.position.data - self.knob_position
        #self.knob_position= data.position.data

        # if self.knob_position_delta != 0 & self.CHANGE_FLAG == False:
        #     self.CHANGE_FLAG = True
        #     print('knob position delta:',  self.knob_position_delta)
        #     print('robot_tcp_position_current:',  self.robot_tcp_position_current)
        #     self.position_change = self.position_factor * self.knob_position_delta
        #     print("Posiiton change,", self.position_change)
        #     self.robot_tcp_position_current[0] = self.robot_tcp_position_current[0] + self.position_change
        #self.POSITION_CHANGED = True

        self.knob_position = data.position.data
        self.knob_force = data.force.data

        self.position_change = self.position_factor * self.knob_position
        self.robot_tcp_position_target[2] = 0.444 - self.position_change

        #print("self.knob_position_delta, self.knob_position, self.knob_force", self.knob_position_delta, self.knob_position, self.knob_force)

    def robot_cartesian_callback(self, data):
        self.robot_tcp_position_current = [round(data.x, 3), round(data.y - 0.005,3), round(data.z,3)] 
        #self.robot_tcp_orientation_current = [round(data.alpha, 3), round(data.beta,3), round(data.gamma,3)]

        self.robot_tcp_position_target = [-0.0000, 0.444, 0.444]
        distance = math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(self.robot_tcp_position_current, self.robot_tcp_position_target)))
        if distance > 0.01:
            self.BOUNDING_MODE = True
        else:
            self.BOUNDING_MODE = False

    def tool_frame_subscribe_callback(self, data) -> None:
        self.robot_tcp_position_current = [round(data.x, 3), round(data.y - 0.005,3), round(data.z,3)] 
        self.robot_tcp_orientation_current = [round(data.alpha, 3), round(data.beta,3), round(data.gamma,3)]
        #print('self.robot_tcp_position_current, self.robot_tcp_orientation_current', self.robot_tcp_position_current, self.robot_tcp_orientation_current)
        self.ROBOT_STATE_INITIALIZED = True

    def sendFRICartesianPose(self) -> bool:
        self.pose.pose.position.z = self.robot_tcp_position_target[2]

        # Publish the Cartesian pose
        self.fri_cartesian_command_publisher.publish(self.pose)
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
            #if self.ROBOT_STATE_INITIALIZED:
                # if self.CHANGE_FLAG == True:
                #     self.robot_tcp_position_target[0] = self.robot_tcp_position_current[0] + self.position_change
                #     self.robot_tcp_position_target[1] = self.robot_tcp_position_current[1]
                #     self.robot_tcp_position_target[2] = self.robot_tcp_position_current[2]
                #     print('robot_tcp_position_target:',  self.robot_tcp_position_target)
                #     self.CHANGE_FLAG = False
                       
            #self.velocity_calculator()

            if not self.BOUNGDING_MODE:
                self.sendFRICartesianPose()
            self.publish_rate.sleep()

if __name__ == '__main__':

    controller = RobotController()
    controller.run()

