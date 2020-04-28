#!/usr/bin/env python

import threading

import numpy as np
import rospy
import serial
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from sensor_msgs.msg import JointState
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformBroadcaster, TransformListener)
from tf.transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_inverse, quaternion_multiply)

pi = 3.14159
pi_2 = pi/2
pi_3 = pi/3
pi_4 = pi/4


class IMU:

    def __init__(self, offset=[0, 0, 0, 1], ref=[0, 0, 0, 1]):
        self.__count = 0
        self.__temp = np.zeros((4,), dtype=np.float64)
        self.__data = np.zeros((4,), dtype=np.float64)
        offset[3] = -offset[3]
        self.__offset_inv = np.array(offset, dtype=np.float64)
        self.__ref = np.array(ref, dtype=np.float64)
        self.__q_offset = quaternion_multiply(
            self.__ref, self.__offset_inv)

    def update(self, new_data):
        self.__count += 1
        self.__temp += new_data

    def read(self):
        return quaternion_multiply(self.__q_offset, self.read_raw())

    def read_raw(self):
        if self.__count > 0:
            self.__data = np.multiply(
                self.__temp, 1.0/self.__count)
            norm = np.linalg.norm(self.__data)
            self.__data /= norm
            self.__count = 0
            self.__temp = np.zeros((4,))
        return self.__data


J1 = IMU([-0.02401736, -0.00586138, -0.81959224,  0.57241363],
         quaternion_from_euler(0, 0, pi_2))
J2 = IMU([-0.00400356,  0.03288636, -0.7436606,  0.66773601],
         quaternion_from_euler(0, 0, pi_2))


def update_imu():
    _serial = serial.Serial("/dev/ttyACM0", 1000000)
    while not rospy.is_shutdown():
        if _serial.readable():
            temp = _serial.readline().rstrip('\n').split(',')
            if len(temp) == 12:
                try:
                    temp = map(int, temp)
                    J1.update(temp[0:4])
                    J2.update(temp[4:8])
                except Exception:
                    pass


if __name__ == "__main__":
    try:
        rospy.init_node("test_quaternion")
        _thread_update_imu = threading.Thread(target=update_imu)
        _thread_update_imu.start()

        br = TransformBroadcaster()
        tf = TransformStamped()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            tf.header.stamp = rospy.Time.now()

            # print(J1.read_raw())
            # print(J2.read_raw())

            q_imu1 = J1.read()
            q_imu2 = J2.read()

            tf.header.frame_id = "base_link"
            tf.child_frame_id = "r_shoulder"
            tf.transform.translation = Vector3(0.119, -0.195, -0.020)
            q = q_imu1
            tf.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
            br.sendTransform(tf)

            tf.header.frame_id = "r_shoulder"
            tf.child_frame_id = "r_elbow"
            tf.transform.translation = Vector3(-0.26, 0.0, 0)
            q = quaternion_inverse(q_imu1)
            q = quaternion_multiply(q, q_imu2)
            tf.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
            br.sendTransform(tf)

            tf.header.frame_id = "r_elbow"
            tf.child_frame_id = "r_hand"
            tf.transform.translation = Vector3(-0.26, 0.0, 0)
            q = quaternion_from_euler(0, 0, 0)
            # q = quaternion_inverse(q_imu2)
            tf.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
            br.sendTransform(tf)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
