import numpy as np
import math


def normalize_theta(theta_matrix):
    theta_matrix = np.where(theta_matrix > math.pi, theta_matrix - 2 * math.pi, theta_matrix)
    theta_matrix = np.where(theta_matrix < -math.pi, theta_matrix + 2 * math.pi, theta_matrix)
    return theta_matrix


def linear_interpolation(x1, y1, x2, y2, x):
    return y1 + (x-x1)*(y2-y1)/(x2-x1)


# 插值计算
def interpolation(interpolate_data: np.array, target_x: np.array, mode="linear"):
    assert len(target_x.shape) == 1
    assert len(interpolate_data.shape) == 2 and interpolate_data.shape[1] == 2
    res = np.zeros((target_x.shape[0], interpolate_data.shape[1]))
    res[:, 0] = target_x

    interpolation_func = None
    if mode == "linear":
        interpolation_func = linear_interpolation

    assert interpolation_func is not None

    for index in range(0, len(res)):
        if res[index, 0] < interpolate_data[0, 0]:
            res[index, 1] = interpolate_data[0, 1]
            continue

        if res[index, 0] > interpolate_data[-1, 0]:
            res[index, 1] = interpolate_data[-1, 1]
            continue

        time_index = np.argwhere(interpolate_data[:, 0] == res[index, 0])
        if time_index.shape == (1, 1):
            res[index, 1] = interpolate_data[time_index[0][0], 1]
            continue

        time_index = np.argwhere(interpolate_data[:, 0] < res[index, 0])
        interpolation_left = np.max(time_index)

        time_index = np.argwhere(interpolate_data[:, 0] > res[index, 0])
        interpolation_right = np.min(time_index)

        res[index, 1] = interpolation_func(interpolate_data[interpolation_left, 0], interpolate_data[interpolation_left, 1],
                                           interpolate_data[interpolation_right, 0], interpolate_data[interpolation_right, 1],
                                           res[index, 0])
    return res


# test
if __name__ == '__main__':
    A = np.array([[1, 2], [2, 4], [3, 6], [4, 8], [8, 16]])
    B = np.array([0, 1, 1.5, 2, 5, 7, 9])
    temp = interpolation(A, B)
    pass
