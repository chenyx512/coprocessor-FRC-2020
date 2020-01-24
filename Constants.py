import numpy as np

class Constants:
    # CV - HSV
    H_MIN = 90
    H_MAX = 100
    S_MIN = 120
    S_MAX = 255
    V_MIN = 47
    V_MAX = 255
    HSV_LOW = (H_MIN, S_MIN, V_MIN)
    HSV_HIGH = (H_MAX, S_MAX, V_MAX)

    # CV - target
    TARGET_3D = np.array([[-0.5 , 0.0   , 0.0],
                          [-0.25, -0.433, 0.0],
                          [0.25 , -0.433, 0.0],
                          [0.5  , 0.0   , 0.0]])
    MIN_TARGET_AREA = 250
    EXTREME_VECTOR = np.array([[-1, -1], [-1, -2], [1, -2], [1, 1]])
    # TopLeft, ButLeft, ButRight, TopRight

    # Connection
    DISCONNECT_TIME = 1.0 # sec

    Debug = False