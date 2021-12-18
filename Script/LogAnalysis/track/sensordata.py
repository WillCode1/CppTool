# odom编码器、imu数据融合，表示相对位姿
import numpy as np
from Tool import common_tool
import matplotlib.pyplot as plt

root = r"D:\workspace\LogAnalysis\log\\"
path = root + r"log_2021-07-16-16-52-24" + r"\\"
data = common_tool.loadtxt2matrix(path + 'sensordata.txt', ' ')

x = data[:, 3]
y = data[:, 4]

plt.plot(x, y, linestyle="-", color='blue', linewidth=1, label="sensordata")
# plt.axis([x.min(), x.max(), y.min(), y.max()])

plt.xlim(x.min(), x.max())
plt.ylim(y.min(), y.max())

plt.xlabel("x")
plt.ylabel("y")
plt.title("sensordata")
plt.legend()
plt.savefig(path + 'sensordata.png')
plt.show()
