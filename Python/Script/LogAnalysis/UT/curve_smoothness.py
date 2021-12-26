# 曲线平滑度
import numpy as np
from Tool import common_tool
import matplotlib.pyplot as plt
from Tool import common_tool as ct


def cal_curve_smoothness(pose_x, pose_y):
    def remove_repeat_point(pose_x, pose_y):
        cur_x = pose_x[0]
        cur_y = pose_y[0]
        for i in range(1, len(pose_x)):
            if cur_x == pose_x[i] and cur_y == pose_y[i]:
                pose_x[i] = pose_y[i] = float('nan')
            else:
                cur_x = pose_x[i]
                cur_y = pose_y[i]

        return pose_x[~np.isnan(pose_x)], pose_y[~np.isnan(pose_y)]

    pose_x, pose_y = remove_repeat_point(pose_x, pose_y)

    delta_y = pose_y[1:] - pose_y[0:-1]
    delta_x = pose_x[1:] - pose_x[0:-1]
    gradient = delta_y / delta_x
    gradient = np.where((delta_x == 0) & (delta_y > 0), float('inf'), gradient)
    gradient = np.where((delta_x == 0) & (delta_y < 0), float('-inf'), gradient)

    radian = np.arctan(gradient)
    delta_radian = np.fabs(radian[1:] - radian[0:-1])

    smoothness = (delta_radian.mean(), np.median(delta_radian), delta_radian.std(), delta_radian.max(), delta_radian.min())
    return smoothness


path = ct.get_desktop_path() + r"UT_log\\"

data_pre = common_tool.loadtxt2matrix(path + 'label_pose.txt', ' ')
x = data_pre[:, 1]
y = data_pre[:, 2]
plt.plot(x, y, linestyle="-", color='blue', linewidth=1, label="label")

data = common_tool.loadtxt2matrix(path + 'absolute_pose.txt', ' ')
x1 = data[:, 1]
y1 = data[:, 2]
plt.plot(x1, y1, linestyle="-", color='red', linewidth=1, label="ekf")

# plt.axis([x.min(), x.max(), y.min(), y.max()])
plt.xlim(x1.min(), x1.max())
plt.ylim(y1.min(), y1.max())
plt.xlabel("x")
plt.ylabel("y")
plt.title("curve_smoothness_compare")
plt.legend()
plt.savefig(path + 'curve_smoothness_compare.png')
plt.show()

data = common_tool.loadtxt2matrix(path + 'relative_pose.txt', ' ')
x2 = data[:, 3]
y2 = data[:, 4]

a = cal_curve_smoothness(x, y)
b = cal_curve_smoothness(x1, y1)
c = cal_curve_smoothness(x2, y2)
print(a)
print(b)
print(c)
