#!/usr/bin/env python2
'''
trans xyzw to rpy and pub to topic: /imu_data
fsf 2019.8.2
'''
import rospy
from math import *
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

rospy.init_node("imu_node")
imu_data_pubber = rospy.Publisher("imu_data", Float32MultiArray, queue_size=10)

def imu_callback(msg):
    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w
    r = atan(2*(w*x+y*z)/(1-2*(x**2+y**2)))
    p = asin(2*(w*y-z*x))
    y = atan(2*(w*z+x*y)/(1-2*(y**2+z**2)))
    pub_msg = Float32MultiArray()
    pub_msg.data = [r, p, y]
    imu_data_pubber.publish(pub_msg)

rospy.Subscriber("imu", Imu, imu_callback)

if __name__ == "__main__":
    rospy.spin()


