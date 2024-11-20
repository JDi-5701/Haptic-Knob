#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import sys
import select
import termios
import tty

# Function to capture keyboard input
def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Main function
def tcp_force_publisher():
    rospy.init_node('tcp_force_controller', anonymous=True)
    pub = rospy.Publisher('tcp_force', Float32, queue_size=10)
    rate = rospy.Rate(10)  # Publish rate in Hz

    tcp_force = 0.0  # Initial value

    rospy.loginfo("TCP Force Controller Node Started")
    rospy.loginfo("Use 'a' to increase by 0.1, 'd' to decrease by 0.1, 's' to reset to 0. Press Ctrl+C to exit.")

    try:
        while not rospy.is_shutdown():
            # Publish the current tcp_force
            pub.publish(tcp_force)

            # Directly call get_key and process the input
            try:
                key = get_key()  # Blocking until key is pressed
                if key == 'a':  # Increase by 0.1
                    tcp_force = min(tcp_force + 0.1, 2.0)  # Clamp to max 2.0
                    rospy.loginfo(f"Increasing: tcp_force = {tcp_force:.2f}")
                elif key == 'd':  # Decrease by 0.1
                    tcp_force = max(tcp_force - 0.1, -2.0)  # Clamp to min -2.0
                    rospy.loginfo(f"Decreasing: tcp_force = {tcp_force:.2f}")
                elif key == 's':  # Reset to 0
                    tcp_force = 0.0
                    rospy.loginfo(f"Reset: tcp_force = {tcp_force:.2f}")
                elif key == '\x03':  # Handle Ctrl+C
                    break
            except IOError:
                rospy.logwarn("Input error detected.")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Shutting down TCP Force Controller Node")

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        tcp_force_publisher()
    except KeyboardInterrupt:
        rospy.loginfo("Force quit detected")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

