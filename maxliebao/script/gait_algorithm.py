#!/usr/bin/env python2
#coding:utf-8

import time
import rospy
import numpy as np
from math import *
from settings import *

def Trot(T, t, legs):
    '''
    return xyz_group:{"RF": [x, y, z],
                      "RB": [x, y, z],
                      "LF": [x, y, z],
                      "LB": [x, y, z] }
    '''
    xyz_group = {leg_name: [0, 0, 0] for leg_name in LEG_NAME}
    t1 = T/2
    t2 = T/2
    step_length = 0.10
    min_height = -0.35
    step_height_up = 0.04
    step_height_down = 0.00
    TROT_OFFSET = [0.00, -0.00, 0.00, -0.00]
    "足端轨迹生成"
    t = t%T
    if t < t1:
        xs = -1./2*step_length*cos(t/t1*pi)
        zs = step_height_up*sin(t/t1*pi)+min_height
    elif (t - t1) < t2:
        xs = 1./2*step_length*cos((t-t1)/t2*pi)
        zs = step_height_down*-sin((t-t1)/t2*pi)+min_height
    xyz_group["RF"] = [xs+TROT_OFFSET[0], 0.02, zs]
    xyz_group["LB"] = [xs+TROT_OFFSET[3], 0.02, zs]
    t_other = t+T/2
    t_other=t_other%T
    if t_other < t1:
        xs = -1./2*step_length*cos(t_other/t1*pi)
        zs = step_height_up*sin(t_other/t1*pi)+min_height
    elif (t_other-t1) < t2:
        xs = 1./2*step_length*cos((t_other-t1)/t2*pi)
        zs = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
    xyz_group["RB"] = [xs+TROT_OFFSET[1], 0.02, zs]
    xyz_group["LF"] = [xs+TROT_OFFSET[2], 0.02, zs]
    return xyz_group

def TrotSlowly(T, t, legs):
    '''
    return xyz_group:{"RF": [x, y, z],
                      "RB": [x, y, z],
                      "LF": [x, y, z],
                      "LB": [x, y, z] }
    '''
    xyz_group = {leg_name: [0, 0, 0] for leg_name in LEG_NAME}
    t1 = T/2
    t2 = T/2
    step_length = 0.03
    min_height = -0.35
    step_height_up = 0.04
    step_height_down = 0.00
    TROT_OFFSET = [0.00, -0.00, 0.00, -0.00]
    "足端轨迹生成"
    t = t%T
    if t < t1:
        xs = -1./2*step_length*cos(t/t1*pi)
        zs = step_height_up*sin(t/t1*pi)+min_height
    elif (t - t1) < t2:
        xs = 1./2*step_length*cos((t-t1)/t2*pi)
        zs = step_height_down*-sin((t-t1)/t2*pi)+min_height
    xyz_group["RF"] = [xs+TROT_OFFSET[0], 0.02, zs]
    xyz_group["LB"] = [xs+TROT_OFFSET[3], 0.02, zs]
    t_other = t+T/2
    t_other=t_other%T
    if t_other < t1:
        xs = -1./2*step_length*cos(t_other/t1*pi)
        zs = step_height_up*sin(t_other/t1*pi)+min_height
    elif (t_other-t1) < t2:
        xs = 1./2*step_length*cos((t_other-t1)/t2*pi)
        zs = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
    xyz_group["RB"] = [xs+TROT_OFFSET[1], 0.02, zs]
    xyz_group["LF"] = [xs+TROT_OFFSET[2], 0.02, zs]
    return xyz_group

def TrotBack(T, t, legs):
    '''
    return xyz_group:{"RF": [x, y, z],
                      "RB": [x, y, z],
                      "LF": [x, y, z],
                      "LB": [x, y, z] }
    '''
    xyz_group = {leg_name: [0, 0, 0] for leg_name in LEG_NAME}
    t1 = T/2
    t2 = T/2
    step_length = 0.05
    min_height = -0.35
    step_height_up = 0.04
    step_height_down = 0.00
    TROT_OFFSET = [0.00, -0.00, 0.00, -0.00]
    "足端轨迹生成"
    t = t%T
    if t < t1:
        xs = -1./2*step_length*cos(t/t1*pi)
        zs = step_height_up*sin(t/t1*pi)+min_height
    elif (t - t1) < t2:
        xs = 1./2*step_length*cos((t-t1)/t2*pi)
        zs = step_height_down*-sin((t-t1)/t2*pi)+min_height
    xyz_group["RF"] = [-(xs+TROT_OFFSET[0]), 0.02, zs]
    xyz_group["LB"] = [-(xs+TROT_OFFSET[3]), 0.02, zs]
    t_other = t+T/2
    t_other=t_other%T
    if t_other < t1:
        xs = -1./2*step_length*cos(t_other/t1*pi)
        zs = step_height_up*sin(t_other/t1*pi)+min_height
    elif (t_other-t1) < t2:
        xs = 1./2*step_length*cos((t_other-t1)/t2*pi)
        zs = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
    xyz_group["RB"] = [-(xs+TROT_OFFSET[1]), 0.02, zs]
    xyz_group["LF"] = [-(xs+TROT_OFFSET[2]), 0.02, zs]
    return xyz_group

def TurnLeft(T, t, legs):
    '''
    return xyz_group:{"RF": [x, y, z],
                      "RB": [x, y, z],
                      "LF": [x, y, z],
                      "LB": [x, y, z] }
    '''
    xyz_group = {leg_name: [0, 0, 0] for leg_name in LEG_NAME}
    t1 = T/2
    t2 = T/2
    step_length = 0.08
    min_height = -0.35
    step_height_up = 0.04
    step_height_down = 0.00
    TROT_OFFSET = [0.00, -0.00, 0.00, -0.00]
    "足端轨迹生成"
    t = t%T
    if t < t1:
        xs = -1./2*step_length*cos(t/t1*pi)
        zs = step_height_up*sin(t/t1*pi)+min_height
    elif (t - t1) < t2:
        xs = 1./2*step_length*cos((t-t1)/t2*pi)
        zs = step_height_down*-sin((t-t1)/t2*pi)+min_height
    xyz_group["RF"] = [xs+TROT_OFFSET[0], 0.02, zs]
    xyz_group["LB"] = [-(xs+TROT_OFFSET[3]), 0.02, zs]
    t_other = t+T/2
    t_other=t_other%T
    if t_other < t1:
        xs = -1./2*step_length*cos(t_other/t1*pi)
        zs = step_height_up*sin(t_other/t1*pi)+min_height
    elif (t_other-t1) < t2:
        xs = 1./2*step_length*cos((t_other-t1)/t2*pi)
        zs = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
    xyz_group["RB"] = [xs+TROT_OFFSET[1], 0.02, zs]
    xyz_group["LF"] = [-(xs+TROT_OFFSET[2]), 0.02, zs]
    return xyz_group

def TurnRight(T, t, legs):
    '''
    return xyz_group:{"RF": [x, y, z],
                      "RB": [x, y, z],
                      "LF": [x, y, z],
                      "LB": [x, y, z] }
    '''
    xyz_group = {leg_name: [0, 0, 0] for leg_name in LEG_NAME}
    t1 = T/2
    t2 = T/2
    step_length = 0.06
    min_height = -0.35
    step_height_up = 0.04
    step_height_down = 0.00
    TROT_OFFSET = [0.00, -0.00, 0.00, -0.00]
    "足端轨迹生成"
    t = t%T
    if t < t1:
        xs = -1./2*step_length*cos(t/t1*pi)
        zs = step_height_up*sin(t/t1*pi)+min_height
    elif (t - t1) < t2:
        xs = 1./2*step_length*cos((t-t1)/t2*pi)
        zs = step_height_down*-sin((t-t1)/t2*pi)+min_height
    xyz_group["RF"] = [-(xs+TROT_OFFSET[0]), 0.02, zs]
    xyz_group["LB"] = [xs+TROT_OFFSET[3], 0.02, zs]
    t_other = t+T/2
    t_other=t_other%T
    if t_other < t1:
        xs = -1./2*step_length*cos(t_other/t1*pi)
        zs = step_height_up*sin(t_other/t1*pi)+min_height
    elif (t_other-t1) < t2:
        xs = 1./2*step_length*cos((t_other-t1)/t2*pi)
        zs = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
    xyz_group["RB"] = [-(xs+TROT_OFFSET[1]), 0.02, zs]
    xyz_group["LF"] = [xs+TROT_OFFSET[2], 0.02, zs]
    return xyz_group

def Idle(T, t, legs):
    return IDLE_STATE

def Squat(legs):
    '''
    Move to Idle state in 1s from current state
    '''
    MovetoP(legs, IDLE_STATE, 0.5, 0.01)

def Jump(legs):
    state1 = {"RF": [-0.05, 0.00, -0.06],
              "RB": [-0.05, 0.00, -0.06],
              "LF": [-0.05, 0.00, -0.06],
              "LB": [-0.05, 0.00, -0.06]}
    MovetoP(legs, state1, 0.5, 0.01)
    time.sleep(0.2)

    state2 = {"RF": [-0.15, 0.00, -0.30],
              "RB": [-0.30, 0.00, -0.30],
              "LF": [-0.15, 0.00, -0.30],
              "LB": [-0.30, 0.00, -0.30]}
    for leg_name in legs:
        legs[leg_name].leg_control(state2[leg_name])
    time.sleep(0.4)

    state3 = {"RF": [ 0.05, 0.30, -0.10],
              "RB": [-0.05, 0.30, -0.10],
              "LF": [ 0.05, 0.30, -0.10],
              "LB": [-0.05, 0.30, -0.10]}
    MovetoP(legs, state3, 0.3, 0.01)

    time.sleep(4.0)

    Squat(legs)

def MovetoP(legs, P, T, step):
    '''
    Each leg from current pos to target pos gradually

    P: {"RF":[x, y, z], "RB":[x, y, z], "LF":[x, y, z], "LB":[x, y, z]}  target_pos
    T: total time
    step: time step
    '''
    start_P = {leg_name:legs[leg_name].end_pos for leg_name in legs}
    step_num = int(T/step)
    step_delta = {leg_name:(np.array(P[leg_name])-start_P[leg_name])/step_num for leg_name in legs}
    for count in range(step_num):
        for leg_name in legs:
            legs[leg_name].leg_control(start_P[leg_name] + count*step_delta[leg_name])
        rospy.sleep(step)

