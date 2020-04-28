#!/usr/bin/env python
import rospy
import numpy as np
import threading
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_inverse
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3

pi = 3.14159


def quaternion_multiply(q1, q0):
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return np.array((
        x1*w0 + y1*z0 - z1*y0 + w1*x0,
        -x1*z0 + y1*w0 + z1*x0 + w1*y0,
        x1*y0 - y1*x0 + z1*w0 + w1*z0,
        -x1*x0 - y1*y0 - z1*z0 + w1*w0), dtype=np.float64)


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


J1 = IMU([0.01400776, 0.01600886, -0.82045425,  0.57131631],
         quaternion_from_euler(0, pi/2, 0))
J2 = IMU([-0.00400189,  0.01000474, -0.137315,    0.99046882],
         quaternion_from_euler(pi/2, 0, pi/2))
J3 = IMU([-0.01601202, 0.02301728, 0.21716304, 0.97573256],
         quaternion_from_euler(pi/2, 0, pi/2))


def update_imu_thread():
    import serial
    _serial = serial.Serial("/dev/ttyACM0", 1000000)
    while not rospy.is_shutdown():
        if _serial.readable():
            temp = _serial.readline().rstrip('\n').split(',')
            if len(temp) == 12:
                try:
                    temp = map(int, temp)
                    J1.update(temp[0:4])
                    J2.update(temp[4:8])
                    J3.update(temp[8:12])
                except Exception:
                    pass


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

    joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    jstate = JointState()
    jstate.name = ['r_shoulder_y_joint', 'r_shoulder_x_joint',
                   'r_shoulder_z_joint', 'r_elbow_y_joint', 'r_elbow_z_joint', 'r_wrist_x_joint']
    # jstate.name = ['l_shoulder_y_joint', 'l_shoulder_x_joint',
    #                'l_shoulder_z_joint', 'l_elbow_y_joint', 'l_elbow_z_joint', 'l_wrist_x_joint']

    rate = rospy.Rate(10)
    z = [0, 0, 0]
    while not rospy.is_shutdown():
        z[0] += 1/50.0
        if z[0] > 1.0:
            z[0] = 1.0
            z[1] += 1/50.0
            if z[1] > 1.0:
                z[1] = 1.0
                z[2] += 1/50.0
                if z[2] > 1.0:
                    z = [0, 0, 0]

        jstate.header.stamp = rospy.Time.now()
        q_imu1 = J1.read()
        q_imu2 = J2.read()
        q_imu3 = J3.read()
        # print(q_imu3)

        # q_imu1 = quaternion_from_euler(0, pi/4, 0, 'rzyx')
        # q_imu2 = quaternion_from_euler(pi/2, pi/2+pi/4, 0, 'rxyz')
        # q_imu3 = quaternion_from_euler(0, 0, 0, 'rxyz')

        q_r_shoulder_fixed = quaternion_from_euler(pi/2, -pi/2, 0)
        q_r_shoulder = q_imu1
        er_s = list(euler_from_quaternion(q_r_shoulder, 'rzyx'))

        q_r_elbow_fixed = quaternion_from_euler(pi/2, 0, pi/2)
        q_r_elbow = quaternion_multiply(q_r_shoulder, q_r_elbow_fixed)
        q_r_elbow = quaternion_multiply(quaternion_inverse(q_r_elbow), q_imu2)
        er_e = list(euler_from_quaternion(q_r_elbow, 'rzyx'))

        q_r_hand_fixed = quaternion_from_euler(0, 0, 0)
        q_hand = quaternion_multiply(q_imu3, quaternion_inverse(q_imu2))
        er_h = list(euler_from_quaternion(q_hand, 'sxyz'))

        try:
            x = np.array(er_s+er_e+er_h) * 180.0/pi
            print(map(int, x))
        except Exception:
            pass

        sendTF("base_link", "r_shoulder_fixed", xyz=[
               0.2, -0.2, 0], q=q_r_shoulder_fixed)
        sendTF("r_shoulder_fixed", "r_shoulder_link",
               xyz=[0, 0, 0], q=quaternion_from_euler(er_s[0], er_s[1], er_s[2]+er_e[0], 'rzyx'))  # q_r_shoulder)
        sendTF("r_shoulder_link", "r_elbow_fixed", xyz=[
               -0.26, 0, 0], q=q_r_elbow_fixed)
        sendTF("r_elbow_fixed", "r_elbow_link",
               xyz=[0, 0, 0], q=quaternion_from_euler(0, er_e[1], er_e[2], 'rzyx'))  # q_r_elbow)  #
        sendTF("r_elbow_link", "r_hand_fixed", xyz=[
            -0.26, 0, 0], q=q_r_hand_fixed)
        sendTF("r_hand_fixed", "r_hand_link", xyz=[
            0.0, 0, 0], q=quaternion_from_euler(0, er_h[2], 0, 'sxyz'))  # q_hand)
        sendTF("r_hand_link", "r_tip_link", xyz=[-0.05, 0, 0])

        # jstate.position = [-er_s[0]+pi, -er_s[1],
        #                    er_s[2]+er_e[0], -er_e[1], er_e[2]+pi, er_h[0]]

        # print(jstate.position)
        # joint_pub.publish(jstate)

        rate.sleep()
