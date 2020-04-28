#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node("test_node")
    rospy.loginfo("Starting test_node.")

    joint_pub = rospy.Publisher("joint_state", JointState, queue_size=10)

    while not rospy.is_shutdown():
        pass
