# odom编码器、imu数据融合，表示相对位姿
import numpy as np
from Tool import common_tool
import matplotlib.pyplot as plt
from Tool import common_tool as ct

path = ct.get_desktop_path() + r"UT_log\\"

data_pre = common_tool.loadtxt2matrix(path + 'pre_relative_pose.txt', ' ')
x = data_pre[:, 3]
y = data_pre[:, 4]
plt.plot(x, y, linestyle="-", color='blue', linewidth=1, label="before")

data = common_tool.loadtxt2matrix(path + 'relative_pose.txt', ' ')
x1 = data[:, 3]
y1 = data[:, 4]
plt.plot(x1, y1, linestyle="-", color='red', linewidth=1, label="after")

# plt.axis([x.min(), x.max(), y.min(), y.max()])
plt.xlim(x.min(), x.max())
plt.ylim(y.min(), y.max())
plt.xlabel("x")
plt.ylabel("y")
plt.title("relative_pose")
plt.legend()
plt.savefig(path + 'relative_pose.png')
plt.show()
