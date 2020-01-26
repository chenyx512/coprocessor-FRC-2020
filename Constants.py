import numpy as np

class Constants:
    # microsoft cam
    EXPOSURE_AUTO = 3 # 1 off 3 on
    EXPOSURE_ABS = 5  # 5-20000, use v4l2 to check
    WIDTH = 640
    HEIGHT = 480
    CAMERA_MATRIX = np.array(
                    [[705.09765532,  0.,           316.48307281],
                    [0.,             705.62034134, 223.3216818],
                    [0.,             0.,           1.]]
    )
    DISTORTION_COEF = np.array([0.1226279, -0.43199166,  0.00196861,
                                -0.00344178, -1.21531281])
    H_MIN = 64
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
    MIN_TARGET_AREA = 25
    MAX_TARGET_DISTANCE = 15
    EXTREME_VECTOR = np.array([[-1, -0.3], [-1, 2], [1, 2], [1, -0.3]])
    # TopLeft, ButLeft, ButRight, TopRight

    # Connection
    DISCONNECT_TIME = 1.0 # sec

    DEBUG = True

# 100, 400; 100 100