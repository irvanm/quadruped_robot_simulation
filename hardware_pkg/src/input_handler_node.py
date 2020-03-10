#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import rospy
from std_msgs.msg import Int8

import time
import math


class InputHandlerNode:

    def __init__(self):
        rospy.init_node('input_handler_node', anonymous=True)
        rospy.on_shutdown(self.shutdown_handler)

        # Setup publisher
        self.command_pub = rospy.Publisher('/direction_command', Int8, queue_size=0)

        # Setup timer events
        self.frequency = 10
        self.process_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.frequency), self.publish_command)
        self.user_input = 0
    
    def publish_command(self, timer_event):
        direction_command = Int8()
        direction_command.data = self.user_input
        self.command_pub.publish(direction_command)
        
    def update_user_input(self, user_input):
        self.user_input = user_input

    def shutdown_handler(self):
        self.user_input = 0
        direction_command = Int8()
        direction_command.data = self.user_input
        self.command_pub.publish(direction_command)

if __name__ == '__main__':
    node = InputHandlerNode()
    try:
        while not rospy.is_shutdown():
            user_input = int(input('Input command: '))
            node.update_user_input(user_input)
            # time.sleep(1)
    except rospy.ROSInterruptException:
        pass

