#!/usr/bin/env python
import threading

import numpy as np
import rospy
import serial
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf.transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_inverse)

pi = 3.14159


def quaternion_multiply(q1, q0):
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return np.array((
        x1*w0 + y1*z0 - z1*y0 + w1*x0,
        -x1*z0 + y1*w0 + z1*x0 + w1*y0,
        x1*y0 - y1*x0 + z1*w0 + w1*z0,
        -x1*x0 - y1*y0 - z1*z0 + w1*w0), dtype=np.float64)


def constrain(amt, low, high):
    return low if (amt < low) else high if (amt > high) else amt


def mapf(x, in_min,  in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


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


J1 = IMU([-0.00900179, -0.03400677,  0.99719846,  0.06601314],
         quaternion_from_euler(0, pi/2, 0))
J2 = IMU([0.00800583, -0.0140102, -0.7075149,  0.70651418],
         quaternion_from_euler(pi/2, 0, pi/2))
J3 = IMU([0.00900613,  0.00700477, -0.92563011,  0.37825749],
         quaternion_from_euler(pi/2, 0, pi/2))

# J4 = IMU([-0.02600555,  0.04500961,  0.99421229,  0.09402008],
#          quaternion_from_euler(0, pi/2, 0))
# J5 = IMU([-0.00500266,  0.02501332, -0.70637625,  0.70737678],
#          quaternion_from_euler(pi/2, 0, pi/2))
# J6 = IMU([-0.05605488, -0.0250245, -0.86985157,  0.48947919],
#          quaternion_from_euler(pi/2, 0, pi/2))

u = [0, 0, 0, 0, 0, 0, 0]
x_cmds = [0, 0, 0, 0, 0, 0, 0]
grip = 0


def update_imu_thread():
    try:
        _serial = serial.Serial("/dev/chinRArm", 1000000)
        while not rospy.is_shutdown():
            if _serial.readable():
                temp = _serial.readline().rstrip('\n').split(',')
                # print(temp)
                if len(temp) == 12:
                    try:
                        temp = map(int, temp)
                        J1.update(temp[0:4])
                        J2.update(temp[4:8])
                        J3.update(temp[8:12])
                    except Exception as e:
                        pass
    except Exception as e:
        print(e)


def update_flex_thread():
    global grip
    try:
        _serial = serial.Serial("/dev/chinRFlex", 115200)
        while not rospy.is_shutdown():
            if _serial.readable():
                temp = _serial.readline()
                grip = int(temp)
    except Exception as e:
        print(e)


def update_arm_thread():
    global u
    try:
        _serial = serial.Serial("/dev/chinRobot", 115200)
        # _serial.write("e\r\n")
        # _serial.write("p\r\n")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            cmds = u
            print(u)
            cmds[0] = constrain(cmds[0], 0, 90)
            cmds[1] = constrain(cmds[1], 0, 90)
            cmds[2] = constrain(mapf(cmds[2], 90, -90, 0, 180), 0, 180)
            cmds[3] = constrain(mapf(cmds[3], -90, 90, 0, 180), 0, 180)
            cmds[4] = constrain(mapf(cmds[4], 0, -180, 0, 180), 0, 180)
            cmds[5] = constrain(mapf(cmds[5], 90, -90, 0, 180), 0, 180)
            print(cmds)
            # print(mapf(cmds[4], 0, -180, 0, 180))

            cmd = "{:03d} {:03d} {:03d} {:03d} {:03d} {:03d} {:03d}\r\n".format(
                cmds[0], cmds[1], cmds[2], cmds[3], cmds[4], 90, 0)
            # cmd = "1{:03d}\r\n".format(cmds[1])
            print(cmd)
            _serial.write(cmd)
            rate.sleep()
    except Exception as e:
        print(e)


br = TransformBroadcaster()


def sendTF(frame_id, child_frame_id, xyz=[0, 0, 0], q=[0, 0, 0, 1]):
    tf = TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation = Vector3(xyz[0], xyz[1], xyz[2])
    tf.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
    br.sendTransform(tf)


if __name__ == "__main__":
    rospy.init_node("imu_to_joint_node")
    rospy.loginfo("Starting imu_to_joint_node.")

    _thread_update_imu = threading.Thread(target=update_imu_thread)
    _thread_update_imu.start()
    _thread_update_flex = threading.Thread(target=update_flex_thread)
    _thread_update_flex.start()
    _thread_update_arm = threading.Thread(target=update_arm_thread)
    _thread_update_arm.start()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # q_imu1 = J1.read_raw()
        # q_imu2 = J2.read_raw()
        # q_imu3 = J3.read_raw()
        # print(q_imu1, q_imu2, q_imu3)
        # rate.sleep()
        # continue

        q_imu1 = J1.read()
        q_imu2 = J2.read()
        q_imu3 = J3.read()
        # q_imu4 = q_imu1
        # q_imu5 = q_imu2
        # q_imu6 = q_imu3

        try:
            q_r_shoulder_fixed = quaternion_from_euler(pi/2, -pi/2, 0)
            q_r_shoulder = q_imu1
            er_s = list(euler_from_quaternion(q_r_shoulder, 'rzyx'))
            el_s = er_s

            q_r_elbow_fixed = quaternion_from_euler(pi/2, 0, pi/2)
            q_r_elbow = quaternion_multiply(q_r_shoulder, q_r_elbow_fixed)
            q_r_elbow = quaternion_multiply(
                quaternion_inverse(q_r_elbow), q_imu2)
            er_e = list(euler_from_quaternion(q_r_elbow, 'rzyx'))
            el_e = er_e

            q_r_hand_fixed = quaternion_from_euler(0, 0, 0)
            q_hand = quaternion_multiply(q_imu3, quaternion_inverse(q_imu2))
            er_h = list(euler_from_quaternion(q_hand, 'sxyz'))
            el_h = er_h

            x = map(int, np.array(er_s+er_e+er_h) * 180.0/pi)
            u = [x[0], x[1], x[2]+x[3], x[4], x[5], x[7], grip]

            q_l_shoulder_fixed = quaternion_from_euler(pi/2, -pi/2, 0)
            q_l_elbow_fixed = quaternion_from_euler(pi/2, 0, pi/2)
            q_l_hand_fixed = quaternion_from_euler(0, 0, 0)

            sendTF("base_link", "r_shoulder_fixed", xyz=[
                0.2, -0.2, 0], q=q_r_shoulder_fixed)
            sendTF("r_shoulder_fixed", "r_shoulder_link",
                   xyz=[0, 0, 0], q=quaternion_from_euler(er_s[0], er_s[1], er_s[2]+er_e[0], 'rzyx'))
            sendTF("r_shoulder_link", "r_elbow_fixed", xyz=[
                -0.26, 0, 0], q=q_r_elbow_fixed)
            sendTF("r_elbow_fixed", "r_elbow_link",
                   xyz=[0, 0, 0], q=quaternion_from_euler(0, er_e[1], er_e[2], 'rzyx'))
            sendTF("r_elbow_link", "r_hand_fixed", xyz=[
                -0.26, 0, 0], q=q_r_hand_fixed)
            sendTF("r_hand_fixed", "r_hand_link", xyz=[
                0.0, 0, 0], q=quaternion_from_euler(0, er_h[2], 0, 'sxyz'))
            sendTF("r_hand_link", "r_tip_link", xyz=[-0.05, 0, 0])

            # sendTF("base_link", "l_shoulder_fixed", xyz=[
            #     0.2, 0.2, 0], q=q_l_shoulder_fixed)
            # sendTF("l_shoulder_fixed", "l_shoulder_link",
            #        xyz=[0, 0, 0], q=quaternion_from_euler(el_s[0], el_s[1], el_s[2]+el_e[0], 'rzyx'))
            # sendTF("l_shoulder_link", "l_elbow_fixed", xyz=[
            #     0.26, 0, 0], q=q_l_elbow_fixed)
            # sendTF("l_elbow_fixed", "l_elbow_link",
            #        xyz=[0, 0, 0], q=quaternion_from_euler(0, el_e[1], el_e[2], 'rzyx'))
            # sendTF("l_elbow_link", "l_hand_fixed", xyz=[
            #     -0.26, 0, 0], q=q_l_hand_fixed)
            # sendTF("l_hand_fixed", "l_hand_link", xyz=[
            #     0.0, 0, 0], q=quaternion_from_euler(0, el_h[2], 0, 'sxyz'))
            # sendTF("l_hand_link", "l_tip_link", xyz=[-0.05, 0, 0])

        except Exception as e:
            print(e)

        rate.sleep()
