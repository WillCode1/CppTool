"""
    因为imu比重较大，接近于融合数据
    所以只比较encode和imu的phi
    绘制delta的导数随时间的变化，以观察是突变还是渐变
"""
# encodephi：odom编码器角度
# imuphi：imu角度yaw
# sensordata：融合后角度
import numpy as np
import math
from Tool import common_tool
from Tool import math_tool as mt
import matplotlib.pyplot as plt


def normalize_theta(theta_matrix, min_=-math.pi, max_=math.pi):
    theta_matrix = np.where(theta_matrix >= max_, theta_matrix - 2 * math.pi, theta_matrix)
    theta_matrix = np.where(theta_matrix < min_, theta_matrix + 2 * math.pi, theta_matrix)
    return theta_matrix


root = r"D:\workspace\LogAnalysis\log\\"
path = root + r"log_2021-10-25-10-46-32" + r"\\"

data_imu = common_tool.loadtxt2matrix(path + 'yaw.txt', ' ', filte_rule=common_tool.filter_yaw)
x_imu = data_imu[:, 1]
y_imu = data_imu[:, 0]
# plt.plot(x_imu, y_imu, linestyle="-", color='red', linewidth=1, label="imu")

# data_fuse = common_tool.loadtxt2matrix(path + 'sensordata.txt', ' ')
# # print(data)
# # print("data's shape", data.shape)
# x_fuse = data_fuse[:, 2]
# y_fuse = data_fuse[:, 5] + (data_imu[0, 0] - data_fuse[0, 5])
# y_fuse = normalize_theta(y_fuse)
# y_fuse = y_fuse * 180 / math.pi
# plt.plot(x_fuse, y_fuse, linestyle="-", color='orange', linewidth=1, label="sensordata")

data_encode = common_tool.loadtxt2matrix(path + 'encodedata.txt', ' ')
x_encode = data_encode[:, 4]
y_encode = data_encode[:, 3] + (data_imu[0, 0] - data_encode[0, 3])
y_encode = normalize_theta(y_encode)
# 插值处理
temp = np.concatenate((np.expand_dims(x_encode, axis=1), np.expand_dims(y_encode, axis=1)), axis=1)
temp = mt.interpolation(temp, x_imu, mode="linear")
x_encode = temp[:, 0]
y_encode = temp[:, 1]

y = y_imu - y_encode
y = normalize_theta(y)
y = y * 180 / math.pi
# plt.plot(x_encode, y_encode, linestyle="-", color='blue', linewidth=1, label="encodephi")
plt.plot(x_imu, y, linestyle="-", color='blue', linewidth=1, label="delta_phi")

plt.xlim(x_imu.min(), x_imu.max())
plt.ylim(y.min(), y.max())

plt.xlabel("time")
plt.ylabel("delta_phi(°c)")
plt.title("delta_phi")
plt.legend()
plt.savefig(path + 'delta_phi.png')
plt.show()
