import rospy
import numpy as np
from math import *
from settings import *
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class MaxLeg():
    def __init__(self, leg_name):
        self.leg_name = leg_name
        self.inverse_flag = -1 if "R" in self.leg_name else 1
        self.joint_state = [0, 0, 0]  # theta1, 2, 3
        self.end_pos = [0, 0, 0]  # x, y, z 
        self.joint_controllers = [0, 0, 0]
        rospy.Subscriber("joint_states", JointState, self.joint_state_subber)
        
        self.joint_1 = 0.0
        self.joint_2 = 0.0
        self.joint_3 = 0.0

        self.joint_controllers[0] = rospy.Publisher(self.leg_name+"1_pos_controller/command", Float64, queue_size=10)
        self.joint_controllers[1] = rospy.Publisher(self.leg_name+"2_pos_controller/command", Float64, queue_size=10)
        self.joint_controllers[2] = rospy.Publisher(self.leg_name+"3_pos_controller/command", Float64, queue_size=10)
        

    def joint_state_subber(self, msg):
        '''
        get joint state from topic /joint_states
        '''
        self.joint_state = np.array(msg.position[JOINT_STATE_TOPIC_CIPHER[self.leg_name]:JOINT_STATE_TOPIC_CIPHER[self.leg_name]+3])
        self.joint_state *= self.inverse_flag
        self.joint_state -= URDF_JOINT_OFFSET
        self.end_pos = Forward(self.joint_state)


    def leg_control(self, xyz):
        thetas = np.array(Inverse(xyz)) + URDF_JOINT_OFFSET
        thetas *= self.inverse_flag
        self.joint_controllers[0].publish(Float64(thetas[0]))
        self.joint_controllers[1].publish(Float64(thetas[1]))
        self.joint_controllers[2].publish(Float64(thetas[2]))


def Inverse(xyz):
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]

    theta1 = atan(y/-z) 
    y = sqrt(y**2 + z**2)
    c3 = (x**2 + y**2 - L2**2 - L3**2) / (2*L2*L3)
    s3 = sqrt(1 - c3**2)
    theta3 = atan2(s3, c3)
    theta2 = atan2(x, y) - atan2(L3*s3, L2 + L3*c3)

    return theta1, -theta2, -theta3

def Forward(thetas):
    theta1 = thetas[0]
    theta2 = thetas[1]
    theta3 = thetas[2]
    x = -L2*sin(theta2) + L3*sin(-theta3-theta2)
    y_ = L2*cos(theta2) + L3*cos(-theta3-theta2)
    z = -sqrt(y_**2/(tan(theta1)**2+1))
    y = -z*tan(theta1)

    return x, y, z

if __name__ == "__main__":
    pass
