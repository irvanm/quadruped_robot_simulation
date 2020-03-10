#!/usr/bin/env python2
#coding:utf-8

import rospy
import time
from std_msgs.msg import String
from utils import MaxLeg
from settings import *
from gait_algorithm import *

class Maxliebao():
    def __init__(self):
        rospy.init_node("Maxliebao_controller_demo")
        self.state = "Idle"
        self.legs = {leg_name: MaxLeg(leg_name) for leg_name in LEG_NAME}
        rospy.Subscriber("state_change", String, self.state_change_callback)

    def main(self): 
        t_start = time.time()
        self.spin_rate = rospy.Rate(100) # 100HZ
        while not rospy.is_shutdown():
            T = GAIT_T_CIPHER[self.state]
            if T != 0:
                t = (time.time() - t_start)%T
                xyz_group = eval(self.state)(T, t, self.legs)
                for leg_name in xyz_group:
                    self.legs[leg_name].leg_control(xyz_group[leg_name])
            else:
                eval(self.state)(self.legs)
                self.state = "Idle"
            self.spin_rate.sleep()
        return

    def state_change_callback(self, msg):
        self.state = msg.data

if __name__ == "__main__":
    Mdog = Maxliebao()
    Mdog.main()
