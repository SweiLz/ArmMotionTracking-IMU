#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from tf2_ros import TransformBroadcaster
from tf.transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_inverse, quaternion_multiply)
from imu_project.msg import arm_robot, arm_sensor
import math
import numpy as np


class AppNode:
    __br = TransformBroadcaster()

    def __init__(self):
        rospy.init_node("application_node")
        rospy.loginfo("Starting AppNode as application_node.")
        self.__visual_rate = rospy.get_param("~visual_rate", default=20)
        self.__command_rate = rospy.get_param("~command_rate", default=10)

        rospy.Subscriber("human_arm_right/sensor",
                         arm_sensor, self.__right_sensor_cb)
        rospy.Subscriber("human_arm_left/sensor",
                         arm_sensor, self.__left_sensor_cb)
        self.__right_sensor = arm_sensor()
        self.__left_sensor = arm_sensor()

        self.__robot_right_pub = rospy.Publisher(
            "robot_arm_right/command", arm_robot, queue_size=10)
        self.__robot_left_pub = rospy.Publisher(
            "robot_arm_left/command", arm_robot, queue_size=10)
        self.__right_robot_arm = arm_robot()
        self.__left_robot_arm = arm_robot()

    def run(self):
        rate = rospy.Rate(self.__visual_rate)
        while not rospy.is_shutdown():
            # Calculate Right IMU to Right hand
            try:
                q_r_shoulder_fixed = quaternion_from_euler(
                    math.pi/2, -math.pi/2, 0)
                q = self.__right_sensor.imu_1
                q_r_shoulder = [q.x, q.y, q.z, q.w]
                er_s = list(euler_from_quaternion(q_r_shoulder, 'rzyx'))

                q_r_elbow_fixed = quaternion_from_euler(
                    math.pi/2, 0, math.pi/2)
                q_r_elbow = quaternion_multiply(
                    q_r_shoulder, q_r_elbow_fixed)
                q = self.__right_sensor.imu_2
                q_r_elbow = quaternion_multiply(
                    quaternion_inverse(q_r_elbow), [q.x, q.y, q.z, q.w])
                er_e = list(euler_from_quaternion(q_r_elbow, 'rzyx'))

                q_r_hand_fixed = quaternion_from_euler(
                    0, 0, 0)
                q_r_hand = quaternion_multiply(
                    [q.x, q.y, q.z, q.w], q_r_hand_fixed)
                q = self.__right_sensor.imu_3
                q_r_hand = quaternion_multiply(
                    quaternion_inverse(q_r_hand), [q.x, q.y, q.z, q.w])
                er_h = list(euler_from_quaternion(q_r_hand, 'rzyx'))

                x = map(int, np.array(er_s+er_e+er_h) * 180.0/math.pi)
                self.__right_robot_arm.joint = [
                    x[0], x[1], x[2] + x[3], x[4], x[5], x[7], self.__right_sensor.hand.data]
                self.__robot_right_pub.publish(self.__right_robot_arm)

                self.__sendTF("base_link", "r_shoulder_fixed", xyz=[
                    0.2, -0.2, 0], q=q_r_shoulder_fixed)
                self.__sendTF("r_shoulder_fixed", "r_shoulder_link",
                              xyz=[0, 0, 0], q=quaternion_from_euler(er_s[0], er_s[1], er_s[2]+er_e[0], 'rzyx'))
                self.__sendTF("r_shoulder_link", "r_elbow_fixed", xyz=[
                    -0.26, 0, 0], q=q_r_elbow_fixed)
                self.__sendTF("r_elbow_fixed", "r_elbow_link",
                              xyz=[0, 0, 0], q=quaternion_from_euler(0, er_e[1], er_e[2], 'rzyx'))
                self.__sendTF("r_elbow_link", "r_hand_fixed", xyz=[
                    -0.26, 0, 0], q=q_r_hand_fixed)
                self.__sendTF("r_hand_fixed", "r_hand_link", xyz=[
                    0.0, 0, 0], q=quaternion_from_euler(0, er_h[1], 0, 'rzyx'))
                self.__sendTF("r_hand_link", "r_tip_link", xyz=[-0.05, 0, 0])

            except Exception as e:
                rospy.logwarn(e)

            try:
                # Calculate IMU to Left hand
                q_l_shoulder_fixed = quaternion_from_euler(
                    math.pi/2, -math.pi/2, 0)
                q = self.__left_sensor.imu_1
                q_l_shoulder = [q.x, q.y, q.z, q.w]
                el_s = list(euler_from_quaternion(q_l_shoulder, 'rzyx'))

                q_l_elbow_fixed = quaternion_from_euler(
                    math.pi/2, 0, math.pi/2)
                q_l_elbow = quaternion_multiply(
                    q_l_shoulder, q_l_elbow_fixed)
                q = self.__left_sensor.imu_2
                q_l_elbow = quaternion_multiply(
                    quaternion_inverse(q_l_elbow), [q.x, q.y, q.z, q.w])
                el_e = list(euler_from_quaternion(q_l_elbow, 'rzyx'))

                q_l_hand_fixed = quaternion_from_euler(0, 0, 0)

                q_l_hand = quaternion_multiply(
                    [q.x, q.y, q.z, q.w], q_r_hand_fixed)
                q = self.__left_sensor.imu_3
                q_l_hand = quaternion_multiply(
                    quaternion_inverse(q_l_hand), [q.x, q.y, q.z, q.w])
                el_h = list(euler_from_quaternion(q_l_hand, 'rzyx'))

                x = map(int, np.array(el_s+el_e+el_h) * 180.0/math.pi)
                self.__left_robot_arm.joint = [
                    x[0], x[1], x[2] + x[3], x[4], x[5], x[7], self.__left_sensor.hand.data]
                self.__robot_left_pub.publish(self.__left_robot_arm)

                self.__sendTF("base_link", "l_shoulder_fixed", xyz=[
                    0.2, 0.2, 0], q=q_l_shoulder_fixed)
                self.__sendTF("l_shoulder_fixed", "l_shoulder_link",
                              xyz=[0, 0, 0], q=quaternion_from_euler(el_s[0], el_s[1], el_s[2]+el_e[0], 'rzyx'))
                self.__sendTF("l_shoulder_link", "l_elbow_fixed", xyz=[
                    0.26, 0, 0], q=q_l_elbow_fixed)
                self.__sendTF("l_elbow_fixed", "l_elbow_link",
                              xyz=[0, 0, 0], q=quaternion_from_euler(0, el_e[1], el_e[2], 'rzyx'))
                self.__sendTF("l_elbow_link", "l_hand_fixed", xyz=[
                    -0.26, 0, 0], q=q_l_hand_fixed)
                self.__sendTF("l_hand_fixed", "l_hand_link", xyz=[
                    0.0, 0, 0], q=quaternion_from_euler(0, el_h[1], 0, 'rzyx'))
                self.__sendTF("l_hand_link", "l_tip_link", xyz=[-0.05, 0, 0])

            except Exception as e:
                rospy.logwarn(e)

            rate.sleep()

    def __right_sensor_cb(self, msg):
        self.__right_sensor = msg

    def __left_sensor_cb(self, msg):
        self.__left_sensor = msg

    def __sendTF(self, frame_id, child_frame_id, xyz=[0, 0, 0], q=[0, 0, 0, 1]):
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = frame_id
        tf.child_frame_id = child_frame_id
        tf.transform.translation = Vector3(xyz[0], xyz[1], xyz[2])
        tf.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        self.__br.sendTransform(tf)


if __name__ == "__main__":
    application_node = AppNode()
    application_node.run()
