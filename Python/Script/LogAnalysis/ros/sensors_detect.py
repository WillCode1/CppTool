#!/usr/bin/env python
# coding = utf-8

from cmath import sqrt
from pickle import TRUE
from re import X
import PyKDL
import math
import rosbag
import traceback
from argparse import ArgumentParser
import numpy as np
import collections
import matplotlib.pyplot as plt
import sys
reload(sys)
sys.setdefaultencoding('utf-8')


class SensorsDetect(object):

    def __init__(self, path, mtype, tfrom, tto):
        self.init_stamp = 0
        self.init_finished = False
        self.odoms = []
        self.imus = []
        self.sfs = []
        self.los = []
        self.ekfs = []
        # self.imu_raw = []
        # self.motor_current = []
        self.hist = collections.defaultdict(list)
        self.read_rosbag(path, mtype, tfrom, tto)

    @staticmethod
    def normalize_angle(angle):
        res = angle
        while res > math.pi:
            res -= 2.0 * math.pi
        while res < -math.pi:
            res += 2.0 * math.pi
        return res

    def process_xy(self, x, y, z, w, px, py, pz, secs, nsecs, lists, nid):
        stamp = secs + float(nsecs) / 1e9
        rot = PyKDL.Rotation.Quaternion(x, y, z, w)
        curr_roll = rot.GetRPY()[0]
        curr_pitch = rot.GetRPY()[1]
        curr_yaw = rot.GetRPY()[2]
        assert type(lists) == list
        if len(lists) == 0:
            lists.append([stamp, 0, 0, curr_yaw, curr_pitch, curr_roll, px, py])
        else:
            angle_diff = self.normalize_angle(curr_yaw - lists[-1][3])
            pos_diff_dx = px - lists[-1][6]
            pos_diff_dy = py - lists[-1][7]
            pos_diff = sqrt(pos_diff_dx * pos_diff_dx + pos_diff_dy * pos_diff_dy) + lists[-1][2]
            # pos_diff = sqrt(pos_diff_dx * pos_diff_dx + pos_diff_dy * pos_diff_dy)
            delta_stamp = stamp - lists[-1][0]
            lists.append([stamp, angle_diff, pos_diff, curr_yaw, curr_pitch, curr_roll, px, py])
            self.hist[nid].append(delta_stamp)
        return lists

    # disable
    def format_imudata(self, gx, gy, gz, ax, ay, az, secs, nsecs, lists):
        stamp = secs + float(nsecs) / 1e9
        lists.append([stamp, gx, gy, gz, ax, ay, az])
        return lists

    @staticmethod
    def sum_diff(sensors):
        for i in range(len(sensors)):
            if i == 0:
                continue
            else:
                sensors[i, 1] = sensors[i, 1] + sensors[i - 1, 1]
        return sensors

    def read_rosbag(self, path, mtype, tfrom, tto):
        imu_topic = ""
        sf_topic = ""
        odom_topic = ""
        lo_topic = ""
        ekf_topic = ""
        if mtype == "run":
            odom_topic = '/peter_motor_core/odom'
            imu_topic = '/imu'
            sf_topic = '/sf'
            lo_topic = '/odom_rf2o'
            ekf_topic = "/robot_pose_ekf/odom"
        elif mtype == "wt":
            odom_topic = '/water_uavcan_master/odom'
            imu_topic = '/water_imu_driver/imu'
            sf_topic = '/sensors_fusion/odom'
            lo_topic = '/odom_rf2o'
        elif mtype == "water":
            odom_topic = '/water_uavcan_master/odom'
            imu_topic = '/water_imu_driver/imu'
            sf_topic = '/sensors_fusion/odom'
            lo_topic = '/odom_rf2o'
        assert imu_topic != ""
        assert odom_topic != ""
        assert sf_topic != ""
        assert lo_topic != ""
        assert ekf_topic != ""
        b = rosbag.Bag(path)
        duration = b.get_end_time() - b.get_start_time()
        start_time = b.get_start_time() + float(tfrom)
        end_time = b.get_start_time() + float(tto)
        summary = b.get_type_and_topic_info(topic_filters=[imu_topic])
        print('imu_hz = ', summary.topics[imu_topic].message_count / duration)
        messages = b.read_messages()
        while True:
            try:
                (topic, msg, t) = next(messages)
                if float(t.secs) > end_time:
                    break
                if float(t.secs) < start_time:
                    continue
                if topic.endswith(odom_topic):
                    self.odoms = self.process_xy(
                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
                        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                        msg.header.stamp.secs, msg.header.stamp.nsecs, self.odoms, odom_topic)

                if topic.endswith(imu_topic):
                    self.imus = self.process_xy(
                        msg.orientation.x, msg.orientation.y,
                        msg.orientation.z, msg.orientation.w, 0, 0, 0,
                        msg.header.stamp.secs, msg.header.stamp.nsecs, self.imus, imu_topic)

                if topic.endswith(sf_topic):
                    self.sfs = self.process_xy(
                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
                        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                        msg.header.stamp.secs, msg.header.stamp.nsecs, self.sfs, sf_topic)

                if topic.endswith(lo_topic):
                    self.los = self.process_xy(
                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
                        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                        msg.header.stamp.secs, msg.header.stamp.nsecs, self.los, lo_topic)

                if topic.endswith(ekf_topic):
                    self.ekfs = self.process_xy(
                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
                        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                        msg.header.stamp.secs, msg.header.stamp.nsecs, self.ekfs, ekf_topic)

                # if topic.endswith('/water_uavcan_master/motor_status'):
                # stamp = msg.motor[0].header.stamp.secs + float(msg.motor[0].header.stamp.nsecs) / 1e9
                # self.motor_current.append([stamp, msg.motor[0].current, msg.motor[1].current])

            except StopIteration:
                break
            except UnicodeDecodeError as e:
                continue
            except Exception as e:
                print('Error message: {0}, args: {1}'.format(e.message, e.args))
                print(traceback.format_exc())

        odoms = np.asarray(self.odoms)
        sfs = np.asarray(self.sfs)
        imus = np.asarray(self.imus)  # format: [stamp, angle_diff, curr_yaw, curr_pitch, curr_roll]
        los = np.asarray(self.los)
        ekfs = np.asarray(self.ekfs)

        n = 180.0 / math.pi

        odomss = self.sum_diff(odoms)
        imuss = self.sum_diff(imus)
        sfss = self.sum_diff(sfs)
        loss = self.sum_diff(los)
        ekfss = self.sum_diff(ekfs)

        plt.figure()
        plt.subplot(2, 1, 1)
        manager = plt.get_current_fig_manager()
        #        manager.full_screen_toggle()
        plt.title(path.split('/')[-1])
        plt.plot(imuss[:, 0] - imuss[0, 0], np.asarray(n * imuss[:, 1]), 'r', label='imu-orientation', linewidth=2)
        plt.plot(odomss[:, 0] - odomss[0, 0], np.asarray(n * odomss[:, 1]), 'b', label='odom-orientation-ab',
                 linewidth=2)
        plt.plot(sfss[:, 0] - sfss[0, 0], np.asarray(n * sfss[:, 1]), 'black', label='sf-orientation', linewidth=2)
        if len(loss) != 0:
            plt.plot(loss[:, 0] - loss[0, 0], np.asarray(n * loss[:, 1]), 'y', label='lo-orientation', linewidth=2)
        if len(ekfss) != 0:
            plt.plot(ekfss[:, 0] - ekfss[0, 0], np.asarray(n * ekfss[:, 1]), 'g', label='ekf-orientation', linewidth=2)

        plt.xlabel('time_stamp(s)')
        plt.legend()
        plt.grid()

        plt.subplot(2, 1, 2)
        #        plt.plot(imuss[:, 0] - imuss[0,0], np.asarray(imuss[:, 2]), 'r', label='imu-positionx', linewidth=2)
        plt.plot(odomss[:, 0] - odomss[0, 0], np.asarray(odomss[:, 2]), 'b', label='odom-positionx-ab', linewidth=0.5)
        plt.plot(sfss[:, 0] - sfss[0, 0], np.asarray(sfss[:, 2]), 'black', label='sf-positionx', linewidth=0.5)
        if len(loss) != 0:
            plt.plot(loss[:, 0] - loss[0, 0], np.asarray(loss[:, 2]), 'y', label='lo-positionx', linewidth=0.5)
        if len(ekfss) != 0:
            plt.plot(ekfss[:, 0] - ekfss[0, 0], np.asarray(ekfss[:, 2]), 'g', label='ekf-positionx', linewidth=0.5)

        plt.xlabel('time_stamp(s)')

        plt.legend()
        plt.grid()
        plt.show()


if __name__ == "__main__":
    try:
        parser = ArgumentParser(description='IMU & Odom-Analysis-Testing Program')
        parser.add_argument('-p', '--path', help='input rosbag path', default='')
        parser.add_argument('-m', '--mtype', help='input type', default='run')
        parser.add_argument('-f', '--tfrom', help='from', default=0.)
        parser.add_argument('-t', '--tto', help='to', default=9999.)
        args = parser.parse_args()
        imu_odom = SensorsDetect(args.path, args.mtype, args.tfrom, args.tto)
    except Exception as e:
        print('Error message: {0}, args: {1}'.format(e.message, e.args))
        print(traceback.format_exc())
