#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import rospy
from std_msgs.msg import Float64
from custom_msgs.msg import GaitCmdMsg

import odrive
from odrive.enums import *
from odrive.utils import start_liveplotter

import time
import math


class HardwareNode:

    def __init__(self):
        rospy.init_node('hardware_node', anonymous=True)
        rospy.on_shutdown(self.shutdown_handler)

        # Setup subscribers
        #self.command_sub = rospy.Subscriber('/gait_cmd', GaitCmdMsg, self.command_callback) #nama topic, tipe msg, interrupt
        self.setpoint_sub =rospy.Subscriber('/RF2_pos_controller/command', Float64, self.command_callback_RF2)
        self.setpoint_sub =rospy.Subscriber('/RF3_pos_controller/command', Float64, self.command_callback_RF3)

        # Setup timer events
        # every 1/10 secs execute self.do_something
        self.frequency = 10
        self.process_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.frequency), self.do_something) #bikin timer interrupt, ngelakuin do something
        
        self.setpoint_RF2 = 0
        self.serpoint_RF3 = 0

        # Find a connected ODrive (this will block until you connect one)
        print("finding an odrive...")
        self.my_drive = odrive.find_any()
        # Find an ODrive that is connected on the serial port /dev/ttyUSB0
        self.my_drive.axis0.controller.pos_setpoint = -1*0.167356116281*(24*8192/6.28)
        self.my_drive.axis0.encoder.set_linear_count(-1*0.167356116281*(24*8192/6.28))
        self.my_drive.axis1.controller.pos_setpoint = -1*0.167356116281*(24*8192/6.28)
        self.my_drive.axis1.encoder.set_linear_count(-6.29654472*(24*8192/6.28))
        
        # Calibrate motor and wait for it to finish
        print("starting calibration...")
        start_liveplotter(lambda:[self.my_drive.axis0.encoder.pos_estimate*360/8192,-1*self.my_drive.axis1.encoder.pos_estimate*360/8192])

        self.my_drive.axis0.encoder.is_ready = True
        self.my_drive.axis1.encoder.is_ready = True
        
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


    def command_callback_RF2(self, msg):
        self.setpoint_RF2 = msg.data
        #self.setpoint1 = ... * msg.LF.joint_2.data 

    def command_callback_RF3(self, msg):
        self.setpoint_RF3 = msg.data
        #self.setpoint1 = ... * msg.LF.joint_2.data 

    
    def do_something(self, timer_event) -> None:
        try:
            self.my_drive.axis0.controller.move_to_pos(-1*self.setpoint_RF2*(24*8192/6.28))
            self.my_drive.axis1.controller.move_to_pos(self.setpoint_RF3*(24*8192/6.28))
        except:
            pass

    
    def shutdown_handler(self):
        self.my_drive.axis0.requested_state = 1
        self.my_drive.axis1.requested_state = 1

if __name__ == '__main__':
    node = HardwareNode()
    try:
        while not rospy.is_shutdown():
            time.sleep(1)
    except rospy.ROSInterruptException:
        pass

