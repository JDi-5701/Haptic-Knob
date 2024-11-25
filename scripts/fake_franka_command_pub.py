#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

def array_to_pose_stamped(pose_array):
    """
    Converts a 1x16 array to a PoseStamped message.
    Args:
        pose_array (list): A 1x16 list representing the pose matrix.
    Returns:
        PoseStamped: The equivalent PoseStamped message.
    """
    if len(pose_array) != 16:
        raise ValueError("Pose array must have exactly 16 elements.")

    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "world"  # You can set the frame ID as appropriate

    # Extract translation (position)
    pose_msg.pose.position.x = pose_array[3]
    pose_msg.pose.position.y = pose_array[7]
    pose_msg.pose.position.z = pose_array[11]
    
    pose_array_4x4 = pose_array.reshape(4, 4)
    r = R.from_matrix(pose_array_4x4[:3, :3])
    
    quaternion = r.as_quat()

    pose_msg.pose.orientation.x = quaternion[0]
    pose_msg.pose.orientation.y = quaternion[1]
    pose_msg.pose.orientation.z = quaternion[2]
    pose_msg.pose.orientation.w = quaternion[3]
    
    return pose_msg

def publish_pose():
    """
    Publishes a manually defined PoseStamped message to the /franka_command topic.
    """
    rospy.init_node('pose_publisher', anonymous=True)
    publisher = rospy.Publisher('/franka_command', PoseStamped, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    # replace this array as the /franka_state_controller/franka_states.O_T_EE
    pose_array = np.array([
        1, 0, 0, 0.5,
        0, 1, 0, 0.5,
        0, 0, 1, 0.5,
        0, 0, 0, 1
    ])

    rospy.loginfo("Publishing PoseStamped messages to /franka_command")
    while not rospy.is_shutdown():
        try:
            pose_msg = array_to_pose_stamped(pose_array)
            publisher.publish(pose_msg)
            rospy.loginfo("Published PoseStamped: %s", pose_msg)
        except ValueError as e:
            rospy.logerr("Error in pose conversion: %s", e)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass

