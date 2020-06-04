#!/usr/bin/env python
import json
import math
import os

import numpy as np
import rospy
import serial
from geometry_msgs.msg import Quaternion
from tf.transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_inverse, quaternion_multiply)

from imu_project.msg import arm_sensor


class IMU:
    def __init__(self, offset=[0, 0, 0, 1], ref=[0, 0, 0, 1]):
        self.__count = 0
        self.__temp = np.zeros((4,), dtype=np.float64)
        self.__data = np.zeros((4,), dtype=np.float64)
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


class SensorNode:
    __IMU_R1 = IMU(ref=quaternion_from_euler(0, math.pi/2, 0))
    __IMU_R2 = IMU(ref=quaternion_from_euler(math.pi/2, 0, math.pi/2))
    __IMU_R3 = IMU(ref=quaternion_from_euler(math.pi/2, 0, math.pi/2))
    __J_R4 = 0.0
    __IMU_L1 = IMU(ref=quaternion_from_euler(0, math.pi/2, 0))
    __IMU_L2 = IMU(ref=quaternion_from_euler(math.pi/2, 0, math.pi/2))
    __IMU_L3 = IMU(ref=quaternion_from_euler(math.pi/2, 0, math.pi/2))
    __J_L4 = 0.0

    def __init__(self):
        rospy.init_node("sensor_node")
        rospy.loginfo("Starting SensorNode as sensor_node.")
        self.__isConfig = rospy.get_param("~config", default=False)
        self.__cfg_file = os.path.join(
            os.path.dirname(__file__), 'config.json')
        self.__port = rospy.get_param("~port", default="/dev/robotIMU")
        self.__serial = None
        self.__publish_rate = rospy.get_param("~rate", default=20)
        self.__isPublish = rospy.get_param("~publish", default=True)

        if self.__isPublish:
            self.__arm_right_pub = rospy.Publisher(
                "human_arm_right/sensor", arm_sensor, queue_size=10)
            self.__arm_left_pub = rospy.Publisher(
                "human_arm_left/sensor", arm_sensor, queue_size=10)
            rospy.Timer(rospy.Duration(
                        1.0/self.__publish_rate), self.__publishSensor)

    def __publishSensor(self, event):
        if not self.__isConfig:
            msg = arm_sensor()

            q_imu = self.__IMU_R1.read()
            msg.imu_1 = Quaternion(
                x=q_imu[0], y=q_imu[1], z=q_imu[2], w=q_imu[3])
            q_imu = self.__IMU_R2.read()
            msg.imu_2 = Quaternion(
                x=q_imu[0], y=q_imu[1], z=q_imu[2], w=q_imu[3])
            q_imu = self.__IMU_R3.read()
            msg.imu_3 = Quaternion(
                x=q_imu[0], y=q_imu[1], z=q_imu[2], w=q_imu[3])
            msg.hand.data = self.__J_R4
            self.__arm_right_pub.publish(msg)

            msg = arm_sensor()
            q_imu = self.__IMU_L1.read()
            msg.imu_1 = Quaternion(
                x=q_imu[0], y=q_imu[1], z=q_imu[2], w=q_imu[3])
            q_imu = self.__IMU_L2.read()
            msg.imu_2 = Quaternion(
                x=q_imu[0], y=q_imu[1], z=q_imu[2], w=q_imu[3])
            q_imu = self.__IMU_L3.read()
            msg.imu_3 = Quaternion(
                x=q_imu[0], y=q_imu[1], z=q_imu[2], w=q_imu[3])
            msg.hand.data = self.__J_L4
            self.__arm_left_pub.publish(msg)

    def __configIMU(self, event):
        rospy.logwarn("IMU Configuring.....")
        for i in range(20):
            self.__IMU_R1.read_raw()
            self.__IMU_R2.read_raw()
            self.__IMU_R3.read_raw()
            self.__IMU_L1.read_raw()
            self.__IMU_L2.read_raw()
            self.__IMU_L3.read_raw()
            rospy.sleep(0.05)
        rospy.sleep(1.0)

        data = {
            'imuR1': list(self.__IMU_R1.read_raw()),
            'imuR2': list(self.__IMU_R2.read_raw()),
            'imuR3': list(self.__IMU_R3.read_raw()),
            'imuL1': list(self.__IMU_L1.read_raw()),
            'imuL2': list(self.__IMU_L2.read_raw()),
            'imuL3': list(self.__IMU_L3.read_raw())
        }
        data_json = json.dumps(data, indent=4)
        with open(self.__cfg_file, 'w') as config_file:
            config_file.write(data_json)
        rospy.logwarn("IMU Configured.")
        self.__loadConfig()
        self.__isConfig = False

    def __loadConfig(self):
        with open(self.__cfg_file, 'r') as config_file:
            cfg_data = json.load(config_file)
            self.__IMU_R1.setOffset(cfg_data["imuR1"])
            self.__IMU_R2.setOffset(cfg_data["imuR2"])
            self.__IMU_R3.setOffset(cfg_data["imuR3"])
            self.__IMU_L1.setOffset(cfg_data["imuL1"])
            self.__IMU_L2.setOffset(cfg_data["imuL2"])
            self.__IMU_L3.setOffset(cfg_data["imuL3"])

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.__serial = serial.Serial(self.__port, 115200)
                rospy.loginfo("IMU Connected.")
                if self.__isConfig:
                    rospy.logwarn("3 second to config.")
                    rospy.sleep(1)
                    rospy.logwarn("2 second to config.")
                    rospy.sleep(1)
                    rospy.logwarn("1 second to config.")
                    rospy.Timer(rospy.Duration(
                        1), self.__configIMU, oneshot=True)
                else:
                    self.__loadConfig()

                rospy.loginfo("Sensor published.")
                while not rospy.is_shutdown():
                    if self.__serial.readable():
                        temp = self.__serial.readline().rstrip('\r\n').split(',')
                        # rospy.loginfo(temp)
                        if len(temp) == 26:
                            try:
                                temp = map(int, temp)
                                if any(temp[0:4]):
                                    self.__IMU_R1.update(temp[0:4])
                                if any(temp[4:8]):
                                    self.__IMU_R2.update(temp[4:8])
                                if any(temp[8:12]):
                                    self.__IMU_R3.update(temp[8:12])
                                self.__J_R4 = temp[12]
                                if any(temp[13:17]):
                                    self.__IMU_L1.update(temp[13:17])
                                if any(temp[17:21]):
                                    self.__IMU_L2.update(temp[17:21])
                                if any(temp[21:25]):
                                    self.__IMU_L3.update(temp[21:25])
                                self.__J_L4 = temp[25]
                            except Exception as e:
                                rospy.logerr(e)

            except Exception as e:
                rospy.logerr("Cannot connect to IMU sensor.")
                rospy.logerr(e)
            rospy.sleep(2.0)


if __name__ == "__main__":
    sensor_node = SensorNode()
    sensor_node.run()
