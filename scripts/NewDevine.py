#!/usr/bin/env python
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rospy
import serial
from tf.transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_inverse)
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from tf2_ros import TransformBroadcaster


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
        # self.__data[3] = 1.0
        offset[3] = -offset[3]
        self.__offset_inv = np.array(offset, dtype=np.float64)
        self.__ref = np.array(ref, dtype=np.float64)
        self.__q_offset = quaternion_multiply(self.__ref, self.__offset_inv)

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

    def setOffset(self, offset):
        offset[3] = -offset[3]
        self.__offset_inv = np.array(offset, dtype=np.float64)
        self.__q_offset = quaternion_multiply(self.__ref, self.__offset_inv)


br = TransformBroadcaster()


def sendTF(frame_id, child_frame_id, xyz=[0, 0, 0], q=[0, 0, 0, 1]):
    tf = TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation = Vector3(xyz[0], xyz[1], xyz[2])
    tf.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
    br.sendTransform(tf)


J1 = IMU(ref=quaternion_from_euler(0, 0, 0))
J2 = IMU(ref=quaternion_from_euler(0, 0, 0))
J3 = IMU(ref=quaternion_from_euler(0, 0, 0))
J4 = 0.0
is_running = False
is_imu_connected = False
is_arm_connected = False
jq = [0, 0, 0, 0, 0, 0, 0]


def update_imu_thread():
    global J1, J2, J3, J4, is_imu_connected, is_running
    try:
        _serial = serial.Serial("/dev/chinRArm", 1000000)
        print("Connected to IMU device.")
        is_imu_connected = True

        while is_running:
            if _serial.readable():
                temp = _serial.readline().rstrip('\n').split(',')
                if len(temp) == 13:
                    try:
                        temp = map(int, temp)
                        if any(temp[0:4]):
                            J1.update(temp[0:4])
                        if any(temp[4:8]):
                            J2.update(temp[4:8])
                        if any(temp[8:12]):
                            J3.update(temp[8:12])
                        J4 = temp[12]
                    except Exception as e:
                        print(e)
    except Exception as e:
        print("Cannot connect to IMU device.")


def update_arm_thread():
    global is_running, is_arm_connected, jq
    try:
        _serial = serial.Serial("/dev/chinRobot", 115200)
        print("Connected to Robot arm.")
        is_arm_connected = True
        _serial.write("e\r\n")
        time.sleep(0.1)
        _serial.write("p\r\n")
        time.sleep(0.1)
        _serial.read(_serial.inWaiting())
        while is_running:
            if any(jq):
                cmds = jq
                print("target -> {}".format(jq))
                cmds[0] = constrain(cmds[0], 0, 90)
                cmds[1] = constrain(cmds[1], 0, 90)
                cmds[2] = constrain(mapf(cmds[2], 90, -90, 0, 180), 0, 180)
                cmds[3] = constrain(mapf(cmds[3], -90, 90, 0, 180), 0, 180)
                cmds[4] = constrain(mapf(cmds[4], 0, -180, 0, 180), 0, 180)
                cmds[5] = constrain(mapf(cmds[5], 90, -90, 0, 180), 0, 180)
                print("command-> {}".format(cmds))
                print("\n")
                # print(mapf(cmds[4], 0, -180, 0, 180))

                # cmds = [0, 30, 90, 0, 90, 90, 0]

                cmd = "{:03d} {:03d} {:03d} {:03d} {:03d} {:03d} {:03d}\r\n".format(
                    cmds[0], cmds[1], cmds[2], cmds[3], cmds[4], cmds[5], 0)
                # cmd = "1{:03d}\r\n".format(cmds[1])
                # print(cmd)
                _serial.write(cmd)
                time.sleep(0.1)
                _serial.read(_serial.inWaiting())
        _serial.write("p\r\n")
        time.sleep(0.1)
        _serial.write("x\r\n")
        time.sleep(0.1)
        _serial.read(_serial.inWaiting())
    except Exception as e:
        print("Cannot connect to Robot arm")


if __name__ == "__main__":
    rospy.init_node("imu_to_joint_node")
    cfg_fname = os.path.join(os.path.dirname(
        __file__), 'config_newdevine.json')

    is_running = True
    _thread_update_imu = threading.Thread(target=update_imu_thread)
    _thread_update_imu.start()
    # _thread_update_arm = threading.Thread(target=update_arm_thread)
    # _thread_update_arm.start()
    time.sleep(1.0)

    if len(sys.argv) > 1:
        cmd = sys.argv[1]
        if cmd == "config":
            if is_imu_connected:
                for i in range(30):
                    J1.read_raw()
                    J2.read_raw()
                    J3.read_raw()
                    time.sleep(0.05)
                time.sleep(1.0)

                data = {'imu1': list(J1.read_raw()), 'imu2': list(
                    J2.read_raw()), 'imu3': list(J3.read_raw())}
                data_json = json.dumps(data, indent=4)
                with open(cfg_fname, 'w') as config_file:
                    config_file.write(data_json)
                print("Save calibration config.")
            else:
                print("IMU device is not connect.")

    else:
        with open(cfg_fname, 'r') as config_file:
            cfg_data = json.load(config_file)
            J1.setOffset(cfg_data["imu1"])
            J2.setOffset(cfg_data["imu2"])
            J3.setOffset(cfg_data["imu3"])
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if is_imu_connected:
                q_imu1 = J1.read()
                q_imu2 = J2.read()
                q_imu3 = J3.read()

                try:
                    # q_r_shoulder_fixed = quaternion_from_euler(
                    #     math.pi/2, -math.pi/2, 0)
                    # q_r_shoulder = q_imu1
                    # er_s = list(euler_from_quaternion(q_r_shoulder, 'rzyx'))
                    # el_s = er_s

                    # q_r_elbow_fixed = quaternion_from_euler(
                    #     math.pi/2, 0, math.pi/2)
                    # q_r_elbow = quaternion_multiply(
                    #     q_r_shoulder, q_r_elbow_fixed)
                    # q_r_elbow = quaternion_multiply(
                    #     quaternion_inverse(q_r_elbow), q_imu2)
                    # er_e = list(euler_from_quaternion(q_r_elbow, 'rzyx'))
                    # el_e = er_e

                    # q_r_hand_fixed = quaternion_from_euler(0, 0, 0)
                    # q_hand = quaternion_multiply(
                    #     q_imu3, quaternion_inverse(q_imu2))
                    # er_h = list(euler_from_quaternion(q_hand, 'sxyz'))
                    # el_h = er_h

                    # x = map(int, np.array(er_s+er_e+er_h) * 180.0/math.pi)
                    # jq = [x[0], x[1], x[2]+x[3], x[4], x[5], x[7], J4]
                    # # print(u)
                    # q_l_shoulder_fixed = quaternion_from_euler(
                    #     math.pi/2, -math.pi/2, 0)
                    # q_l_elbow_fixed = quaternion_from_euler(
                    #     math.pi/2, 0, math.pi/2)
                    # q_l_hand_fixed = quaternion_from_euler(0, 0, 0)

                    sendTF("base_link", "test_relative_imu1", xyz=[
                        0, 0, 0], q=q_imu1)
                    sendTF("base_link", "test_relative_imu2", xyz=[
                        0, 0, 0.05], q=q_imu2)
                    q_diff = quaternion_multiply(
                        q_imu1, quaternion_inverse(q_imu2))
                    sendTF("base_link", "test_relative_delta", xyz=[
                        0, 0, 0.1], q=q_diff)

                    print(euler_from_quaternion(q_diff)[2]*180/math.pi)
                    # sendTF("r_shoulder_fixed", "r_shoulder_link",
                    #        xyz=[0, 0, 0], q=quaternion_from_euler(er_s[0], er_s[1], er_s[2]+er_e[0], 'rzyx'))
                    # sendTF("r_shoulder_link", "r_elbow_fixed", xyz=[
                    #     -0.26, 0, 0], q=q_r_elbow_fixed)
                    # sendTF("r_elbow_fixed", "r_elbow_link",
                    #        xyz=[0, 0, 0], q=quaternion_from_euler(0, er_e[1], er_e[2], 'rzyx'))
                    # sendTF("r_elbow_link", "r_hand_fixed", xyz=[
                    #     -0.26, 0, 0], q=q_r_hand_fixed)
                    # sendTF("r_hand_fixed", "r_hand_link", xyz=[
                    #     0.0, 0, 0], q=quaternion_from_euler(0, er_h[2], 0, 'sxyz'))
                    # sendTF("r_hand_link", "r_tip_link", xyz=[-0.05, 0, 0])

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
    is_running = False
