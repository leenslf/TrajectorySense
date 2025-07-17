#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import sys

def send_elevation_loop(elevation_cm):
    # Clamp elevation between 0 and 60 cm
    elevation_cm = max(0.0, min(60.0, elevation_cm))
    elevation_m = elevation_cm / 100.0  # convert to meters

    # Create the JointState message
    joint_msg = JointState()
    joint_msg.name = ['j1', 'j2', 'j3', 'j4']
    joint_msg.position = [elevation_m] * 4  # same elevation for all joints
    joint_msg.velocity = [0.14] * 4         # small holding velocity
    joint_msg.effort = [0.0] * 4            # effort is typically left as 0

    rate = rospy.Rate(10)  # 10 Hz
    rospy.loginfo(f"Holding elevation at {elevation_cm} cm ({elevation_m} m)... Press Ctrl+C to stop.")

    while not rospy.is_shutdown():
        joint_msg.header.stamp = rospy.Time.now()
        pub.publish(joint_msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('table_elevation_controller')
    pub = rospy.Publisher(f'{sys.argv[1]}/joint_command', JointState, queue_size=10)
    rospy.sleep(1.0)  # give publisher time to connect

    try:
        elevation = float(sys.argv[2])
    except ValueError:
        print("Invalid elevation value. Must be a number.")
        sys.exit(1)

    try:
        send_elevation_loop(elevation)
    except rospy.ROSInterruptException:
        rospy.loginfo("Elevation command stopped.")
