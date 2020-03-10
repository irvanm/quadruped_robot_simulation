#!/usr/bin/env python2
'''
To deal with laser data, find the dune distance and pub to topic: /dune_distane
the X axis of laser refer to 0 degree, Hokuyo's scan range is [-135, 135]
fsf 2019.8.2
'''
import rospy
from math import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

rospy.init_node("laser_node")
dune_distance_pubber = rospy.Publisher("dune_distance", Float32, queue_size=10)

def duneDistanceDetect(ranges = [], scan_scope = [-90.0, 90.0], msg=""):
    '''
    find closest dune x distance and return
    '''
    ranges_partial = ranges[len(ranges)/2+int(scan_scope[0]/(msg.angle_increment/pi*180)) -1:
                            len(ranges)/2+int(scan_scope[1]/(msg.angle_increment/pi*180)) ]
    ranges_partial = reversed(ranges_partial)
    xys = []
    for index,distance in enumerate(ranges_partial):
        x = distance * cos((scan_scope[1]/180*pi)-index*msg.angle_increment)
        y = distance * sin((scan_scope[1]/180*pi)-index*msg.angle_increment)
        # find the first y of dune 
        if abs(y) >= 0.16 and abs(y) <= 0.18:
            return x
    return -1

def laser_callback(msg):
    ranges = msg.ranges
    pub_msg = Float32(duneDistanceDetect(ranges,[0.0,60.0],msg))
    dune_distance_pubber.publish(pub_msg)
    
rospy.Subscriber("laser/scan",LaserScan,laser_callback)

if __name__ == "__main__":
    rospy.spin()
