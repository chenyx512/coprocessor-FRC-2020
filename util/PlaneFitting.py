# https://github.com/htcr/plane-fitting
import numpy as np
import numpy.linalg as la

eps = 0.00001


def fit_plane_LSE(points):
    # points: Nx4 homogeneous 3d points
    # return: 1d array of four elements [a, b, c, d] of
    # ax+by+cz+d = 0
    assert points.shape[0] >= 3  # at least 3 points needed
    U, S, Vt = svd(points)
    null_space = Vt[-1, :]
    return null_space


def get_point_dist(points, plane):
    # return: 1d array of size N (number of points)
    dists = np.abs(points @ plane) / np.sqrt(
        plane[0] ** 2 + plane[1] ** 2 + plane[2] ** 2)
    return dists


def fit_plane_LSE_RANSAC(points_3d, max_iters=10, inlier_thresh=0.4):
    """
    :param points: N * 3 points
    :param max_iters:
    :param inlier_thresh:
    :return:
    """
    SAMPLE_SIZE = 4000

    points = np.ones((len(points_3d), 4))
    points[:, :3] = points_3d
    if(len(points) > SAMPLE_SIZE):
        index = np.random.choice(len(points), SAMPLE_SIZE, replace=False)
        points = points[index]

    max_inlier_num = -1
    max_inlier_list = None
    N = points.shape[0]
    assert N >= 3

    for i in range(max_iters):
        chose_id = np.random.choice(N, 3, replace=False)
        chose_points = points[chose_id, :]
        tmp_plane = fit_plane_LSE(chose_points)

        dists = get_point_dist(points, tmp_plane)
        tmp_inlier_list = np.where(dists < inlier_thresh)[0]
        tmp_inliers = points[tmp_inlier_list, :]
        num_inliers = tmp_inliers.shape[0]
        if num_inliers > max_inlier_num:
            max_inlier_num = num_inliers
            max_inlier_list = tmp_inlier_list
            final_plane = tmp_plane

    # print(len(max_inlier_list) / len(points_3d))
    # final_points = points[max_inlier_list, :]
    # plane = fit_plane_LSE(final_points)
    return final_plane


def svd(A):
    u, s, vh = la.svd(A)
    S = np.zeros(A.shape)
    S[:s.shape[0], :s.shape[0]] = np.diag(s)
    return u, S, vh


def inverse_sigma(S):
    inv_S = S.copy().transpose()
    for i in range(min(S.shape)):
        if abs(inv_S[i, i]) > eps:
            inv_S[i, i] = 1.0 / inv_S[i, i]
    return inv_S


def svd_solve(A, b):
    U, S, Vt = svd(A)
    inv_S = inverse_sigma(S)
    svd_solution = Vt.transpose() @ inv_S @ U.transpose() @ b
    return svd_solution
