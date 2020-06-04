#!/usr/bin/env python
import rospy
import serial
from imu_project.msg import arm_robot


class RobotNode:
    def __init__(self):
        rospy.init_node("robot_node", disable_signals=True)
        rospy.loginfo("Starting RobotNode as robot_node.")

        self.__port_right = rospy.get_param(
            "~port_right", default="/dev/robotArmR")
        self.__port_left = rospy.get_param(
            "~port_left", default="/dev/robotArmL")

        self.__robot_arm_right = None
        self.__robot_arm_left = None

        try:
            self.__robot_arm_right = serial.Serial(self.__port_right, 115200)
            rospy.loginfo("Connected to robot right arm.")
            self.__robot_arm_right.write("e\r\n")
            rospy.sleep(0.1)
            self.__robot_arm_right.write("p\r\n")
            rospy.sleep(0.1)
            self.__robot_arm_right.read(self.__robot_arm_right.inWaiting())
        except Exception as e:
            rospy.logwarn(e)

        try:
            self.__robot_arm_left = serial.Serial(self.__port_left, 115200)
            rospy.loginfo("Connected to robot left arm.")
            self.__robot_arm_left.write("e\r\n")
            rospy.sleep(0.1)
            self.__robot_arm_left.write("p\r\n")
            rospy.sleep(0.1)
            self.__robot_arm_left.read(self.__robot_arm_left.inWaiting())

        except Exception as e:
            rospy.logwarn(e)

        rospy.Subscriber("robot_arm_right/command",
                         arm_robot, self.__right_command_cb)
        rospy.Subscriber("robot_arm_left/command",
                         arm_robot, self.__left_command_cb)

    def __right_command_cb(self, msg):
        rospy.loginfo("Right val: {}".format(msg))

        if len(msg.joint) == 7:
            cmds = map(int, msg.joint)

            ########## Mapping to robot joint ##########
            cmds[0] = self.constrain(cmds[0], 0, 90)
            cmds[1] = self.constrain(cmds[1], 0, 90)
            cmds[2] = self.constrain(
                self.mapf(cmds[2], 90, -90, 0, 180), 0, 180)
            cmds[3] = self.constrain(
                self.mapf(cmds[3], -90, 90, 0, 180), 0, 180)
            cmds[4] = self.constrain(
                self.mapf(cmds[4], 0, -180, 0, 180), 0, 180)
            cmds[5] = self.constrain(
                self.mapf(cmds[5], 90, -90, 0, 180), 0, 180)
            ############################################

            cmd = "{:03d} {:03d} {:03d} {:03d} {:03d} {:03d} {:03d}\r\n".format(
                cmds[0], cmds[1], cmds[2], cmds[3], cmds[4], cmds[5], 0)
            if self.__robot_arm_right != None:
                self.__robot_arm_right.write(cmd)
                rospy.sleep(0.1)
                self.__robot_arm_right.read(self.__robot_arm_right.inWaiting())
            else:
                rospy.loginfo("Right cmd: {}".format(cmd))
        else:
            rospy.logwarn("array index not match")

    def __left_command_cb(self, msg):
        rospy.loginfo("Left val: {}".format(msg))
        if len(msg.joint) == 7:
            cmds = map(int, msg.joint)

            ########## Mapping to robot joint ##########
            if cmds[0] < 0:
                cmds[0] = cmds[0]+180  # constrain(cmds[0], 0, 90)
            elif cmds[0] > 90:
                cmds[0] = 0
            else:
                cmds[0] = 180

            cmds[1] = self.constrain(cmds[1], 0, 90)

            if cmds[2] < 0:
                cmds[2] += 270
            else:
                cmds[2] -= 90
            cmds[2] = self.constrain(cmds[2], 0, 180)

            # constrain(mapf(cmds[3], -90, 90, 0, 180), 0, 180)
            cmds[3] = -cmds[3]+90
            # constrain(mapf(cmds[4], 0, -180, 0, 180), 0, 180)
            if cmds[4] < 0:
                cmds[4] = 270 + cmds[4]
            else:
                cmds[4] = cmds[4] - 90

            # constrain(mapf(cmds[5], 90, -90, 0, 180), 0, 180)
            cmds[5] = 90
            ############################################

            cmd = "{:03d} {:03d} {:03d} {:03d} {:03d} {:03d} {:03d}\r\n".format(
                cmds[0], cmds[1], cmds[2], cmds[3], cmds[4], cmds[5], 0)
            if self.__robot_arm_left != None:
                self.__robot_arm_left.write(cmd)
                rospy.sleep(0.1)
                self.__robot_arm_left.read(self.__robot_arm_left.inWaiting())
            else:
                rospy.loginfo("Left cmd: {}".format(cmd))

        else:
            rospy.logwarn("array index not match")

    def run(self):
        try:
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
            # rospy.spin()
        except KeyboardInterrupt:
            if self.__robot_arm_right != None:
                self.__robot_arm_right.write("p\r\n")
                rospy.sleep(0.1)
                self.__robot_arm_right.write("x\r\n")
                rospy.sleep(0.1)
                self.__robot_arm_right.read(self.__robot_arm_right.inWaiting())
            if self.__robot_arm_left != None:
                self.__robot_arm_left.write("p\r\n")
                rospy.sleep(0.1)
                self.__robot_arm_left.write("x\r\n")
                rospy.sleep(0.1)
                self.__robot_arm_left.read(self.__robot_arm_left.inWaiting())
            print("End command")

    @staticmethod
    def constrain(amt, low, high):
        return low if (amt < low) else high if (amt > high) else amt

    @staticmethod
    def mapf(x, in_min,  in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


if __name__ == "__main__":
    robot_node = RobotNode()
    robot_node.run()
