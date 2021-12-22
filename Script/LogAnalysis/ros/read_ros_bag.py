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


if __name__ == '__main__':
    bag = bagreader(path + 'ekf_pose.bag')
    # preview_data(df_laser)

    odom_msg = bag.message_by_topic('/peter_motor_core/odom')
    odom = pd.read_csv(odom_msg)
    x = odom['pose.pose.position.x']
    y = odom['pose.pose.position.y']
    plt.plot(x, y, linestyle="-", color='blue', linewidth=1, label="odom")

    lo_msg = bag.message_by_topic('/odom_rf2o')
    lo = pd.read_csv(lo_msg)
    x = lo['pose.pose.position.x']
    y = lo['pose.pose.position.y']
    x = x + odom['pose.pose.position.x'][0] - x[0]
    y = y + odom['pose.pose.position.y'][0] - y[0]
    plt.plot(x, y, linestyle="-", color='red', linewidth=1, label="lo")

    sf_msg = bag.message_by_topic('/sf')
    sf = pd.read_csv(sf_msg)
    x = sf['pose.pose.position.x']
    y = sf['pose.pose.position.y']
    x = x + odom['pose.pose.position.x'][0] - x[0]
    y = y + odom['pose.pose.position.y'][0] - y[0]
    plt.plot(x, y, linestyle="-", color='gray', linewidth=1, label="sf")

    ekf_msg = bag.message_by_topic('/robot_pose_ekf/odom')
    ekf = pd.read_csv(ekf_msg)
    x = ekf['pose.pose.position.x']
    y = ekf['pose.pose.position.y']
    x = x + odom['pose.pose.position.x'][0] - x[0]
    y = y + odom['pose.pose.position.y'][0] - y[0]
    plt.plot(x, y, linestyle="-", color='orange', linewidth=1, label="ekf")

    # plt.axis([x.min(), x.max(), y.min(), y.max()])
    plt.xlim(x.min(), x.max())
    plt.ylim(y.min(), y.max())
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("track_compare")
    plt.legend()
    # plt.savefig(path + 'fuse.png')
    plt.show()

