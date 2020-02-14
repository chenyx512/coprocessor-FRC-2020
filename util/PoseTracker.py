import numpy as np
import math as m
from queue import Queue, Full
import logging

T265_X = np.array([[-0.03], [0.15]])
T265_THETA = 90
CV_X = np.array([[-0.35], [-0.02]])
CV_THETA = 180
QUEUE_MAX_SIZE = 4


class PoseTracker:
    # x_field = R(dtheta_r2f) * x_robot + dx_r2f

    def __init__(self):
        self.x_robot = np.zeros([2, 1])
        self.theta_robot = 0
        self.x_field = np.zeros([2, 1])
        self.theta_field = 0
        self.dtheta_r2f = 0
        self.dx_r2f = np.zeros([2, 1])
        self.dtheta_r2f_q = Queue(maxsize=QUEUE_MAX_SIZE)
        self.dx_r2f_q = Queue(maxsize=QUEUE_MAX_SIZE)
        self.calibration_error_x = 1e9
        self.calibration_error_theta = 1e9
        self.logger = logging.getLogger(__name__)

    def update_t265_pose(self, x, y, theta):
        x_t265_in_t265 = np.array([[x], [y]])
        x_robot_in_t265 = x_t265_in_t265 - np.matmul(_rot_mat(theta + T265_THETA),
                                                     -T265_X)
        self.theta_robot = theta
        self.x_robot = np.matmul(_rot_mat(T265_THETA), x_robot_in_t265) \
                       + T265_X
        self.theta_field = theta + self.dtheta_r2f
        self.x_field = np.matmul(_rot_mat(self.dtheta_r2f), self.x_robot) \
                       + self.dx_r2f

    def update_CV_in_field(self, x, y, theta):
        x_CV_in_field = np.array([[x], [y]])
        x_robot_in_field = x_CV_in_field - np.matmul(_rot_mat(theta + CV_THETA),
                                                     -CV_X)
        # self.logger.info(f"x {x_robot_in_field} theta {theta}")
        theta_robot_in_field = theta - CV_THETA
        self.calibration_error_x = np.linalg.norm(x_robot_in_field - self.x_field)
        self.calibration_error_theta = m.fabs(self.theta_field - theta_robot_in_field)

        dtheta_r2f = theta_robot_in_field - self.theta_robot
        _put_and_rotate(dtheta_r2f, self.dtheta_r2f_q)
        dx_r2f = x_robot_in_field - \
                np.matmul(_rot_mat(dtheta_r2f), self.x_robot)
        _put_and_rotate(dx_r2f, self.dx_r2f_q)

    def calibrate(self):
        if not self.dx_r2f_q.qsize() == self.dtheta_r2f_q.qsize() == QUEUE_MAX_SIZE:
            self.logger.warning("fail to calibrate because queue not full")
            return False
        self.dtheta_r2f = np.median(_dump_queue_to_list(self.dtheta_r2f_q))
        self.dx_r2f = np.median(_dump_queue_to_list(self.dx_r2f_q), axis=0)
        return True

    def clear_calibration(self):
        _dump_queue_to_list(self.dx_r2f_q)
        _dump_queue_to_list(self.dtheta_r2f_q)

    @property
    def field_xyt(self):
        return self.x_field.flatten().tolist() + [self.theta_field % 360]

    @property
    def robot_xyt(self):
        return self.x_robot.flatten().tolist() + [self.theta_robot % 360]


def _put_and_rotate(element, queue):
    try:
        queue.put_nowait(element)
    except Full:
        queue.get_nowait()
        queue.put_nowait(element)


def _dump_queue_to_list(queue):
    values = []
    while not queue.empty():
        values.append(queue.get_nowait())
    return values


def _rot_mat(theta):
    """
    :param theta: degree in angle
    :return: rotation matrix 2 * 2
    """
    theta = m.radians(theta)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    return R
