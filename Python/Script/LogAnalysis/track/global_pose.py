# 融合label数据，表示绝对位姿
import numpy as np
from Tool import common_tool
import matplotlib.pyplot as plt

root = r"D:\workspace\LogAnalysis\log\\"
path = root + r"log_2021-07-16-16-52-24" + r"\\"
data = common_tool.loadtxt2matrix(path + 'global_pose.txt', ' ')

x = data[:, 1]
y = data[:, 2]

plt.plot(x, y, linestyle="-", color='blue', linewidth=1, label="global_pose")
# plt.axis([x.min(), x.max(), y.min(), y.max()])
plt.xlim(x.min(), x.max())
plt.ylim(y.min(), y.max())
plt.xlabel("x")
plt.ylabel("y")
plt.title("global_pose")
plt.legend()
plt.savefig(path + 'global_pose.png')
plt.show()
