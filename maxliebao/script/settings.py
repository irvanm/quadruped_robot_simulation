LEG_NAME = ["RF", "RB", "LF", "LB"]
JOINT_STATE_TOPIC_CIPHER = {"RF": 9, "RB": 6, "LF": 3, "LB": 0}
URDF_JOINT_OFFSET = [0.0, -0.64, 1.63]
GAIT_T_CIPHER = {"Idle": 1.0,
                 "Trot" : 0.5,
                 "TrotSlowly" : 0.5,
                 "TrotBack" : 0.5,
                 "TurnLeft" : 0.5,
                 "TurnRight" : 0.5,
                 "Jump" : 0.,   # 0 means this gait has nothing to do with T or t
                 "Squat": 0.,
                 } 

GAIT_COMMANDER_CIPHER = {"i": "Idle",
                         "t": "Trot",
                         "ts": "TrotSlowly",
                         "tb": "TrotBack",
                         "tl": "TurnLeft",
                         "tr": "TurnRight",
                         "j": "Jump",
                         "s": "Squat",
                        }
L1 = 0.0722
L2 = 0.24
L3 = 0.21
IDLE_STATE = {"RF": [ 0.05, 0.02, -0.35],
              "RB": [-0.05, 0.02, -0.35],
              "LF": [ 0.05, 0.02, -0.35],
              "LB": [-0.05, 0.02, -0.35]}
