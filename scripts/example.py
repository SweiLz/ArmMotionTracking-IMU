#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray


def joint_cb(msg):
    rospy.loginfo("Joint target {}".format(msg.data))


if __name__ == "__main__":
    rospy.init_node("example_node")
    rospy.loginfo("Starting example_node.")

    rospy.Subscriber("joint_target", Float32MultiArray, joint_cb)

    while not rospy.is_shutdown():
        pass
