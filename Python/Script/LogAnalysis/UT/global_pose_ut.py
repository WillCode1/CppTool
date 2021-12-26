# 融合label数据，表示绝对位姿
import numpy as np
from Tool import common_tool
import matplotlib.pyplot as plt
from Tool import common_tool as ct

path = ct.get_desktop_path() + r"UT_log\\"

data_pre = common_tool.loadtxt2matrix(path + 'pre_absolute_pose.txt', ' ')
x = data_pre[:, 1]
y = data_pre[:, 2]
plt.plot(x, y, linestyle="-", color='blue', linewidth=1, label="before")

data = common_tool.loadtxt2matrix(path + 'absolute_pose.txt', ' ')
x1 = data[:, 1]
y1 = data[:, 2]
plt.plot(x1, y1, linestyle="-", color='red', linewidth=1, label="after")

# plt.axis([x.min(), x.max(), y.min(), y.max()])
plt.xlim(x.min(), x.max())
plt.ylim(y.min(), y.max())
plt.xlabel("x")
plt.ylabel("y")
plt.title("absolute_pose")
plt.legend()
plt.savefig(path + 'absolute_pose.png')
plt.show()
