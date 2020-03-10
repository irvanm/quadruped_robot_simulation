#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import rospy
from std_msgs.msg import Int8
from custom_msgs.msg import GaitCmdMsg

import time
import math


class middleware_node:

    def __init__(self):
        rospy.init_node('middleware_node', anonymous=True)
        rospy.on_shutdown(self.shutdown_handler)
        #setup publisher
        #self.command_pub = rospy.Publisher('/gait_cmd', GaitCmdMsg, queue_size=0)

        # setup subscriber
        self.command_sub = rospy.Subscriber('/direction_command', Int8, self.command_callback)
        
        # Setup timer events
        self.frequency = 10
        self.publish_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.frequency), self.publish_command)
        self.process_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.frequency), self.calculate_motor_setpoint)

        self.offset_RB2 = 2.3
        self.offset_RB3 = 1.0
        self.offset_LB2 = 3.9
        self.offset_LB3 = 5.2
        self.offset_RF2 = 2.3
        self.offset_RF3 = 1.0
        self.offset_LF2 = 3.9
        self.offset_LF3 = 5.2
        self.L = 0.4

        self.robot_movement_velocity = 0.1
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.current_velocity_x_command = 0.0
        self.current_velocity_y_command = 0.0
        
        self.user_input = 0

        self.LF_joint_1 = 0
        self.LF_joint_2 = 0
        self.LF_joint_3 = 0
        self.RF_joint_1 = 0
        self.RF_joint_2 = 0
        self.RF_joint_3 = 0
        self.LB_joint_1 = 0
        self.LB_joint_2 = 0
        self.LB_joint_3 = 0
        self.RB_joint_1 = 0
        self.RB_joint_2 = 0
        self.RB_joint_3 = 0

        

    def command_callback(self, msg):
        self.command = msg.data
        self.update_body_setpoint()


    def update_body_setpoint(self):
        if self.command == 1:
            # up
            # self.setpoint_x = 0.0
            self.setpoint_y = 0.5
            # self.current_velocity_x_command = 0.0
            if self.current_velocity_y_command < 0.5:
                self.current_velocity_y_command += self.robot_movement_velocity / self.frequency
            else:
                self.current_velocity_y_command = 0.5
        elif self.command == 2:
            # down
            # self.setpoint_x = 0.0
            self.setpoint_y = 0.0
            # self.current_velocity_x_command = 0.0
            if self.current_velocity_y_command > 0.0:
                self.current_velocity_y_command -= self.robot_movement_velocity / self.frequency
            else:
                self.current_velocity_y_command = 0.0
        elif self.command == 3:
            # forward
            self.setpoint_x = 0.5
            # self.setpoint_y = 0.0
            # self.current_velocity_y_command = 0.0
            if self.current_velocity_x_command < 0.5:
                self.current_velocity_x_command += self.robot_movement_velocity / self.frequency
            else:
                self.current_velocity_x_command = 0.5
        elif self.command == 4:
            # forward
            self.setpoint_x = 0.0
            # self.setpoint_y = 0.0
            # self.current_velocity_y_command = 0.0
            if self.current_velocity_x_command > 0.0:
                self.current_velocity_x_command -= self.robot_movement_velocity / self.frequency
            else:
                self.current_velocity_x_command = 0.0    

    def calculate_motor_setpoint(self, timer_event):
        self.a1 = (self.current_velocity_x_command**2 + self.current_velocity_y_command**2 ) / 2*self.L - 1
        self.a2 = self.current_velocity_x_command*self.L*(1+math.cos(self.RF_joint_3)) + self.current_velocity_y_command * self.L * math.sin(self.RF_joint_3)
        self.a3 = self.current_velocity_x_command**2 + self.current_velocity_y_command**2

        if self.a3 == 0:
            self.a3 = 0.1

        # ngitung 12 motor
        self.RF_joint_1 = 0
        self.LF_joint_1 = 0
        self.RB_joint_1 = 0
        self.LB_joint_1 = 0

        self.RF_joint_3 = (self.offset_RF3 + self.a1)
        self.LF_joint_3 = 1.57 + self.offset_LF3 + self.a1
        self.RB_joint_3 = (self.offset_RB3 + self.a1)
        self.LB_joint_3 = 1.57 + self.offset_LF3 + self.a1
        
        self.RF_joint_2 = 1.57 - (self.offset_RF2 + self.a2/self.a3)
        self.LF_joint_2 = 3.14 - (self.offset_RB2 + self.a2/self.a3)
        self.RB_joint_2 = 1.57 + self.offset_LF2 + self.a2/self.a3
        self.LB_joint_2 = 3.14 + self.offset_LB2 + self.a2/self.a3

        
        

    def publish_command(self, timer_event):
        gait_msg = GaitCmdMsg()
        gait_msg.LF.joint_1 = self.LF_joint_1
        gait_msg.LF.joint_2 = self.LF_joint_2
        gait_msg.LF.joint_3 = self.LF_joint_3
        gait_msg.RF.joint_1 = self.RF_joint_1
        gait_msg.RF.joint_2 = self.RF_joint_2
        gait_msg.RF.joint_3 = self.RF_joint_3
        gait_msg.LB.joint_1 = self.LB_joint_1
        gait_msg.LB.joint_2 = self.LB_joint_2
        gait_msg.LB.joint_3 = self.LB_joint_3
        gait_msg.RB.joint_1 = self.RB_joint_1
        gait_msg.RB.joint_2 = self.RB_joint_2
        gait_msg.RB.joint_3 = self.RB_joint_3
        self.command_pub.publish(gait_msg)
        
    def shutdown_handler(self):
        pass



if __name__ == '__main__':
    node = middleware_node()
    try:
        while not rospy.is_shutdown():
            time.sleep(1)
    except rospy.ROSInterruptException:
        pass

