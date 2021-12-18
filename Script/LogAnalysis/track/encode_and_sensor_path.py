"""
    比较融合轨迹、编码器轨迹
"""
import numpy as np
from Tool import common_tool as ct
from Tool import matrix_tool as mt
import matplotlib.pyplot as plt

root = r"D:\workspace\LogAnalysis\log\\"
path = root + r"log_2021-08-23-11-45-29" + r"\\"

data_encode = ct.loadtxt2matrix(path + 'encodedata.txt', ' ')
x_encode = data_encode[:, 1]
y_encode = data_encode[:, 2]
plt.plot(x_encode, y_encode, linestyle="-", color='blue', linewidth=1, label="encodedata")

data_fuse = ct.loadtxt2matrix(path + 'sensordata.txt', ' ')
xy_fuse = data_fuse[:, 3:5]
# print(xy_fuse.shape)e
# 坐标转换
delta_theta = data_encode[0, 3] - data_fuse[0, 5]
xy_fuse = mt.coordinate_transformations_matrix2d(xy_fuse, mt.rotation_matrix2d(delta_theta))
delta_xy = data_encode[0, 1:3] - xy_fuse[0]
xy_fuse = mt.coordinate_transformations_matrix2d(xy_fuse, mt.translation_matrix2d(delta_xy))

x_fuse = xy_fuse[:, 0]
y_fuse = xy_fuse[:, 1]
plt.plot(x_fuse, y_fuse, linestyle="-", color='red', linewidth=1, label="sensordata")

# plt.xlim(x_encode.min(), x_encode.max())
# plt.ylim(y_encode.min(), y_encode.max())

plt.xlabel("x")
plt.ylabel("y")
plt.title("track_compare")
plt.legend()
plt.savefig(path + 'track_compare.png')
plt.show()
