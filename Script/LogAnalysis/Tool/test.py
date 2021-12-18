import numpy as np
import matplotlib.pyplot as plt

# https://blog.csdn.net/Frank_LJiang/article/details/89363901
x = np.linspace(0.05, 10, 1000)
y = np.cos(x)

plt.plot(x, y, linestyle="-", color='red', linewidth=1, label="sensordata")

# plt.xlim(x_encode.min(), x_encode.max())
# plt.ylim(y_encode.min(), y_encode.max())

plt.xlabel("x")
plt.ylabel("y")
plt.title("functon")
plt.legend()
plt.show()
