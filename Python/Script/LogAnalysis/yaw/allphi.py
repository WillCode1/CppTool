"""
    所有phi比较
"""
# encodephi：odom编码器角度
# imuphi：imu角度yaw
# sensordata：融合后角度
import numpy as np
import math
from Tool import common_tool
import matplotlib.pyplot as plt


def normalize_theta(theta_matrix):
    theta_matrix = np.where(theta_matrix > math.pi, theta_matrix - 2 * math.pi, theta_matrix)
    theta_matrix = np.where(theta_matrix < -math.pi, theta_matrix + 2 * math.pi, theta_matrix)
    return theta_matrix


root = r"D:\workspace\LogAnalysis\log\\"
path = root + r"log_2021-10-25-10-46-32" + r"\\"

data_imu = common_tool.loadtxt2matrix(path + 'yaw.txt', ' ', filte_rule=common_tool.filter_yaw)
x_imu = data_imu[:, 1]
y_imu = data_imu[:, 0]*180/math.pi
plt.plot(x_imu, y_imu, linestyle="-", color='red', linewidth=1, label="imu")

data_fuse = common_tool.loadtxt2matrix(path + 'sensordata.txt', ' ')
# print(data)
# print("data's shape", data.shape)
x_fuse = data_fuse[:, 2]
y_fuse = data_fuse[:, 5] + (data_imu[0, 0] - data_fuse[0, 5])
y_fuse = normalize_theta(y_fuse)
y_fuse = y_fuse * 180 / math.pi
plt.plot(x_fuse, y_fuse, linestyle="-", color='orange', linewidth=1, label="sensordata")

data_encode = common_tool.loadtxt2matrix(path + 'encodedata.txt', ' ')
x_encode = data_encode[:, 4]
y_encode = data_encode[:, 3] + (data_imu[0, 0] - data_encode[0, 3])
y_encode = normalize_theta(y_encode)
y_encode = y_encode * 180 / math.pi
plt.plot(x_encode, y_encode, linestyle="-", color='blue', linewidth=1, label="encodephi")

plt.xlim(x_imu.min(), x_imu.max())
plt.ylim(-180, 180)

plt.xlabel("time")
plt.ylabel("phi")
plt.title("allphi")
plt.legend()
plt.savefig(path + 'allphi.png')
plt.show()
