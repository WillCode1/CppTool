import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd

path = '~/catkin_ws/src/robot_pose_ekf/test/'
b = bagreader(path + 'ekf_pose.bag')

# replace the topic name as per your need
LASER_MSG = b.message_by_topic('/robot_pose_ekf/odom_combined')
print(LASER_MSG)
df_laser = pd.read_csv(LASER_MSG)
print(df_laser)
