import numpy as np
import math


# 二维矩阵运算
def translation_matrix2d(translation_vector: np.array):
    assert translation_vector.shape == (2,)
    res = np.eye(3)
    res[2, 0] = translation_vector[0]
    res[2, 1] = translation_vector[1]
    return res


def rotation_matrix2d(rotation_angle: float):
    res = np.eye(3)
    res[0, 0] = math.cos(rotation_angle)
    res[0, 1] = math.sin(rotation_angle)
    res[1, 0] = -math.sin(rotation_angle)
    res[1, 1] = math.cos(rotation_angle)
    return res


def scaling_matrix2d(scale_vector: np.array):
    assert scale_vector.shape == (2,)
    res = np.eye(3)
    res[0, 0] = scale_vector[0]
    res[1, 1] = scale_vector[1]
    return res


# 向量左乘矩阵
def coordinate_transformations_matrix2d(coord2d: np.array, mat2d: np.array):
    # print(coord2d.shape)
    assert mat2d.shape == (3, 3)

    is_single = False
    if coord2d.shape == (2,):
        temp = np.ones((1,))
        coord2d = np.concatenate((coord2d, temp), axis=0)
        is_single = True
    elif len(coord2d.shape) == 2 and coord2d.shape[1] == 2:
        temp = np.ones((coord2d.shape[0], 1))
        coord2d = np.concatenate((coord2d, temp), axis=1)
    else:
        assert False

    # print(coord2d.shape)
    res = np.dot(coord2d, mat2d)
    if is_single:
        return res[0:2]
    else:
        return res[:, 0:2]


# test
if __name__ == '__main__':
    matrix1 = translation_matrix2d(np.array([1, 2]))
    print(matrix1)
    matrix2 = rotation_matrix2d(0.5 * math.pi)
    print(matrix2)
    matrix3 = scaling_matrix2d(np.array([2, 2]))
    print(matrix3)
    print(np.matmul(matrix2, matrix1))
    print(np.dot(matrix2, matrix1))

    print(coordinate_transformations_matrix2d(np.array([[2, 2], [3, 3]]), matrix1))
    # print(coordinate_transformations_matrix2d(np.array([2, 2]), matrix1))
