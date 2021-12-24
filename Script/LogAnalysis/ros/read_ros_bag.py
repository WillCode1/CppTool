"""
    比较轨迹
"""
from bagpy import bagreader
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
from Tool import matrix_tool as mt

path = '/home/diwei/catkin_ws/src/robot_pose_ekf/test/'


def preview_data(pandas_data):
    # 表头
    print(pandas_data.head())
    # 表信息描述
    print(pandas_data.info())
    # DataFrame统计信息
    print(pandas_data.describe())
    # 绘制统计图
    pandas_data.hist(bins=50, figsize=(20, 15))
    plt.show()


x_range = [np.inf, -np.inf]
y_range = [np.inf, -np.inf]


def fix_range(x, y):
    if x.min() < x_range[0]:
        x_range[0] = x.min()
    if x.max() > x_range[1]:
        x_range[1] = x.max()
    if y.min() < y_range[0]:
        y_range[0] = y.min()
    if y.max() > y_range[1]:
        y_range[1] = y.max()


def draw_track_by_topic(ros_bag, topic: str, color: str, label: str, reference=None, need_rotate=True):
    msg_file = ros_bag.message_by_topic(topic)
    data = pd.read_csv(msg_file)
    x = data['pose.pose.position.x']
    y = data['pose.pose.position.y']
    if reference is not None:
        coord = np.stack((x.values, y.values), axis=-1)

        # 坐标转换
        # a = math.acos(data['pose.pose.orientation.w'][0]) * 2
        # print(a)
        # b = math.acos(reference['pose.pose.orientation.w'][0]) * 2
        # print(b)
        # print(a-b)
        # print('======')

        if need_rotate:
            rotation = mt.rotation_matrix2d((math.acos(data['pose.pose.orientation.w'][0]) -
                                             math.acos(reference['pose.pose.orientation.w'][0])) * 2)
            coord = mt.coordinate_transformations_matrix2d(coord, rotation)

        trans_vector = np.array([reference['pose.pose.position.x'][0],
                                 reference['pose.pose.position.y'][0]])-coord[0]
        translation = mt.translation_matrix2d(trans_vector)
        coord = mt.coordinate_transformations_matrix2d(coord, translation)

        x, y = coord[:, 0], coord[:, 1]
    plt.plot(x, y, linestyle="-", color=color, linewidth=1, label=label)
    fix_range(x, y)
    return data


if __name__ == '__main__':
    bag = bagreader(path + 'ekf_pose.bag')
    # preview_data(bag)

    odom = draw_track_by_topic(bag, '/peter_motor_core/odom', 'blue', 'odom')
    draw_track_by_topic(bag, '/odom_rf2o', 'red', 'lo', odom)
    draw_track_by_topic(bag, '/sf', 'gray', 'sf', odom)
    draw_track_by_topic(bag, '/robot_pose_ekf/odom', 'orange', 'ekf', odom)

    # plt.axis([x.min(), x.max(), y.min(), y.max()])
    plt.xlim(x_range[0], x_range[1])
    plt.ylim(y_range[0], y_range[1])
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("track_compare")
    plt.legend()
    # plt.savefig(path + 'track_compare.png')
    plt.show()

