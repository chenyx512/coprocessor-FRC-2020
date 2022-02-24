import math
import numpy as np

def sphereFit(points):
    #   Assemble the A matrix
    spX = points[:, 0]
    spY = points[:, 1]
    spZ = points[:, 2]
    A = np.zeros((len(spX),4))
    A[:,0] = spX*2
    A[:,1] = spY*2
    A[:,2] = spZ*2
    A[:,3] = 1

    #   Assemble the f matrix
    f = np.zeros((len(spX),1))
    f[:,0] = (spX*spX) + (spY*spY) + (spZ*spZ)
    C, residules, rank, singval = np.linalg.lstsq(A,f)

    #   solve for the radius
    t = (C[0]*C[0])+(C[1]*C[1])+(C[2]*C[2])+C[3]
    radius = math.sqrt(t)

    return radius, C[0].item(), C[1].item(), C[2].item()


def get_point_dist(points, sphere):
    r, x, y, z = sphere
    dis = (points - np.array([x, y, z])) ** 2
    dis = np.sum(dis, axis=-1) ** 0.5
    return abs(dis - r)


def fit_sphere_LSE_RANSAC(points, max_iters=80, inlier_thresh=0.015):
    """
    :param points: N * 3 points
    :param max_iters:
    :param inlier_thresh:
    :return:
    """
    SAMPLE_SIZE = 1000

    if(len(points) > SAMPLE_SIZE):
        index = np.random.choice(len(points), SAMPLE_SIZE, replace=False)
        points = points[index]

    max_inlier_num = -1
    max_inlier_list = None
    N = points.shape[0]
    if N <= 250:
        return 0, (0, 0, 0, 0)

    for i in range(max_iters):
        chose_id = np.random.choice(N, 4, replace=False)
        chose_points = points[chose_id, :]
        tmp_sphere = sphereFit(chose_points)

        dists = get_point_dist(points, tmp_sphere)
        tmp_inlier_list = np.where(dists < inlier_thresh)[0]
        tmp_inliers = points[tmp_inlier_list, :]
        num_inliers = tmp_inliers.shape[0]
        if num_inliers > max_inlier_num:
            max_inlier_num = num_inliers
            max_inlier_list = tmp_inlier_list
            final_sphere = tmp_sphere

    final_sphere = sphereFit(points[max_inlier_list])
    return len(max_inlier_list) / len(points), final_sphere
