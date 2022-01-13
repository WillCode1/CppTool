"""
    比较轨迹
"""
from bagpy import bagreader
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
from Script.LogAnalysis.Tool import matrix_tool as mt
from Script.LogAnalysis.Tool import angle_tool as at


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


def draw_track_by_topic(ros_bag, topic: str, color: str, label: str, reference=None, positive=True):
    msg_file = ros_bag.message_by_topic(topic)
    data = pd.read_csv(msg_file)
    x = data['pose.pose.position.x']
    y = data['pose.pose.position.y']
    if reference is not None:
        coord = np.stack((x.values, y.values), axis=-1)

        # 坐标转换
        if positive:
            radian = (math.acos(reference['pose.pose.orientation.w'][0]) -
                      math.acos(data['pose.pose.orientation.w'][0])) * 2
        else:
            radian = (math.acos(reference['pose.pose.orientation.w'][0]) +
                      math.acos(data['pose.pose.orientation.w'][0])) * 2

        rotation = mt.rotation_matrix2d(radian)
        coord = mt.coordinate_transformations_matrix2d(coord, rotation)

        print(label, at.radian2angle(2 * math.acos(data['pose.pose.orientation.w'][0])),
                     at.radian2angle(2 * math.acos(reference['pose.pose.orientation.w'][0])),
                     at.radian2angle(2 * math.acos(reference['pose.pose.orientation.w'][0]) +
                                     2 * math.acos(data['pose.pose.orientation.w'][0])))

        trans_vector = np.array([reference['pose.pose.position.x'][0],
                                 reference['pose.pose.position.y'][0]])-coord[0]
        translation = mt.translation_matrix2d(trans_vector)
        coord = mt.coordinate_transformations_matrix2d(coord, translation)

        x, y = coord[:, 0], coord[:, 1]
    plt.plot(x, y, linestyle="-", color=color, linewidth=1, label=label)
    fix_range(x, y)
    return data


if __name__ == '__main__':
    is_test = True
    # is_test = False

    if is_test:
        path = '/home/diwei/.ros/'
        bag = bagreader(path + 'ekf_pose_record.bag')
    else:
        path = '/home/diwei/catkin_ws/src/robot_pose_ekf/bagfiles/'
        # bag = bagreader(path + 'oneloop/8309_2022-01-03-21-01-39.bag')
        bag = bagreader(path + 'oneloop/front_desk_1F_1_2022-01-03-21-04-03.bag')
    # preview_data(bag)

    sf = draw_track_by_topic(bag, '/sf', 'gray', 'sf')
    draw_track_by_topic(bag, '/peter_motor_core/odom', 'blue', 'odom', sf)
    draw_track_by_topic(bag, '/odom_rf2o', 'red', 'lo', sf, False)
    if is_test:
        draw_track_by_topic(bag, '/robot_pose_ekf/odom', 'orange', 'ekf', sf)
    # else:
        # draw_track_by_topic(bag, '/robot_pose_ekf/odom', 'orange', 'ekf', odom)
        draw_track_by_topic(bag, '/map_server/robot_pose', 'black', 'global', sf)

    # plt.axis([x.min(), x.max(), y.min(), y.max()])
    plt.xlim(x_range[0], x_range[1])
    plt.ylim(y_range[0], y_range[1])
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("track_compare")
    plt.legend()
    plt.savefig(path + 'track_compare.png')
    plt.show()

