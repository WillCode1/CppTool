"""
    比较轨迹
"""
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt

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


def draw_track_by_topic(ros_bag, topic: str, color: str, label: str, reference=None):
    msg_file = ros_bag.message_by_topic(topic)
    data = pd.read_csv(msg_file)
    x = data['pose.pose.position.x']
    y = data['pose.pose.position.y']
    if reference is not None:
        x = x + reference['pose.pose.position.x'][0] - x[0]
        y = y + reference['pose.pose.position.y'][0] - y[0]
    plt.plot(x, y, linestyle="-", color=color, linewidth=1, label=label)
    return data


if __name__ == '__main__':
    bag = bagreader(path + 'ekf_pose.bag')
    # preview_data(bag)

    odom = draw_track_by_topic(bag, '/peter_motor_core/odom', 'blue', 'odom')
    draw_track_by_topic(bag, '/odom_rf2o', 'red', 'lo', odom)
    draw_track_by_topic(bag, '/sf', 'gray', 'sf', odom)
    draw_track_by_topic(bag, '/robot_pose_ekf/odom', 'orange', 'ekf', odom)

    x = odom['pose.pose.position.x']
    y = odom['pose.pose.position.y']
    # plt.axis([x.min(), x.max(), y.min(), y.max()])
    plt.xlim(x.min(), x.max())
    plt.ylim(y.min(), y.max())
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("track_compare")
    plt.legend()
    # plt.savefig(path + 'track_compare.png')
    plt.show()

