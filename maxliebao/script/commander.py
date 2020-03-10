#!/usr/bin/env python2
#coding:utf-8

import rospy
import time
import numpy as np
from math import *
from std_msgs.msg import String, Float32, Float32MultiArray
from settings import GAIT_COMMANDER_CIPHER

class Commander():
    def __init__(self):
        rospy.init_node("commander")
        self.command_pubber = rospy.Publisher("state_change", String, queue_size = 10)
        rospy.Subscriber("imu_data", Float32MultiArray, self.imu_callback)
        rospy.Subscriber("dune_distance", Float32, self.dune_distance_callback)
        self.imu_data = [0., 0., 0.]
        self.dune_distance = 0.

    def main_loop(self):
        while not rospy.is_shutdown():
            print("Commander:")
            for command in GAIT_COMMANDER_CIPHER:
                print(command+"\t: "+GAIT_COMMANDER_CIPHER[command])
            print("auto\t: Automatic workflow")
            print("q\t: exit")
            command = raw_input()
            if command in GAIT_COMMANDER_CIPHER:
                self.command_pubber.publish(GAIT_COMMANDER_CIPHER[command])
            elif command == "auto":
                self.Pub_command("t", 4.0)
                while self.imu_data[2] < 38:
                    self.Pub_command("tl", 0.01)
                while abs(self.dune_distance) > 0.3:
                    self.Pub_command("ts", 0.01)
                self.Pub_command("j", 7)
                while self.imu_data[2] < 80:
                    self.Pub_command("tl", 0.01)
                self.Pub_command("t", 1)
                while abs(self.dune_distance) > 0.9:
                    self.Pub_command("ts", 0.01)
                while abs(self.imu_data[2]) > 20:
                    self.Pub_command("tr", 0.01)
                self.Pub_command("t", 4)
                while abs(self.dune_distance) > 0.6:
                    self.Pub_command("ts", 0.01)
                while abs(self.imu_data[2]) < 88:
                    self.Pub_command("tr", 0.01)
                self.Pub_command("ts", 13)
                while abs(self.imu_data[1]) > 5:
                    self.Pub_command("t", 0.01)
                self.Pub_command("t", 1)
                self.Pub_command("s", 1)
            elif command == 'q':
                exit()

    def Pub_command(self, command, T):
        self.command_pubber.publish(GAIT_COMMANDER_CIPHER[command])
        time.sleep(T)

    def imu_callback(self, msg):
        self.imu_data = np.array(msg.data)/pi*180

    def dune_distance_callback(self, msg):
        self.dune_distance = msg.data

if __name__ == "__main__":
    mCommander = Commander()
    mCommander.main_loop()
