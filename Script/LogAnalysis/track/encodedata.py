# odom编码器，表示相对位姿
import numpy as np
from Tool import common_tool
import matplotlib.pyplot as plt

root = r"D:\workspace\LogAnalysis\log\\"
path = root + r"log_2021-07-16-16-52-24" + r"\\"
data = common_tool.loadtxt2matrix(path + 'encodedata.txt', ' ')
# print(data)
# print("data's shape", data.shape)

x = data[:, 1]
y = data[:, 2]

plt.plot(x, y, linestyle="-", color='blue', linewidth=1, label="encodedata")
# plt.axis([x.min(), x.max(), y.min(), y.max()])

plt.xlim(x.min(), x.max())
plt.ylim(y.min(), y.max())

plt.xlabel("x")
plt.ylabel("y")
plt.title("encodedata")
plt.legend()
plt.savefig(path + 'encodedata.png')
plt.show()
