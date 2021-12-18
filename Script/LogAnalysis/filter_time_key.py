import os
import re
import time
import shutil
from Tool import common_tool as ct


class FilterLog:
    def __init__(self):
        pass

    @staticmethod
    def check_line(line: str, regular_expression: list) -> bool:
        for index, value in enumerate(regular_expression):
            if re.search(value, line):
                return True

    def check_file(self, src_file: str, dest_file: str, regular_expression: list):
        _, file_name = ct.split_file_path_and_name(src_file)
        file_path, _ = ct.split_file_path_and_name(dest_file)

        read_file = None
        try:
            read_file = open(src_file, 'r', errors='ignore', encoding='utf-8')
            all_lines = read_file.readlines()
            read_file.close()
        except FileNotFoundError:
            print("{0}: doesn't exist!".format(file_name))
            return
        except UnicodeDecodeError:
            read_file.close()
            print("{0}: read fail!".format(file_name))
            return

        filter_res = []
        for index, value in enumerate(all_lines):
            if self.check_line(value, regular_expression):
                filter_res.append(value)

        # if not os.path.exists(file_path):
        #     os.makedirs(file_path)

        content = ''.join(filter_res)
        write_file = open(dest_file, 'w', encoding='utf-8')
        write_file.write(content)
        write_file.close()


def regular_expression_by_key(time: str, key: str) -> str:
    return r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ ' + '{0}'.format(time) + '\:\d{2} \d+\]){1}.*' + '{0} '.format(key)


def regular_expressions_by_key(start_time: str, end_time: str, key: str) -> str:
    return "temp"


if __name__ == '__main__':
    # output_path = r"/work/robot/Bin/Arm-Release/"
    # src_file = r"/work/robot/Bin/Arm-Release/" + r"log"

    # src_file = ct.get_desktop_path() + r"imuerror\log_backup1\log_backup1"
    # src_file = r"D:\workspace\LabelCheck\labelchecklog\log7"
    src_file = ct.get_desktop_path() + r"log"
    output_path = ct.get_desktop_path()
    # pose_file = output_path + r"global_pose.txt"
    # sensordata_file = output_path + r"sensordata.txt"

    tool = FilterLog()

    key_word = '.*'
    regular_expression = []
    # regular_expression.append(regular_expression_by_key('15:20', key_word))
    # regular_expression.append(regular_expression_by_key('15:30', key_word))
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} imu_error!')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} mapCheckDis1_')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} mapCheckDis2_')

    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} sendSpeedDown35ms spd')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} smoothSpeed->setSpeedDst:')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} copyPoseBufData 338 test1')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} copyPoseBufData 346 test2')

    # regular_expression.append(r'getRobotPose')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} tryMoveWithOutGazer')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} (fuse|forcast) cur_robot_pose:')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} fuse label_pose:')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} insert 3 sensordata:')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} setEncodeData')

    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} setLabelData 766 pose')
    # regular_expression.append(r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} checkLabel 1826')

    regular_expression.append(r'label check')
    regular_expression.append(r'label check 2')
    regular_expression.append(r'label check 5')
    # regular_expression.append(r'label check 6')
    # regular_expression.append(r'label check 7')
    regular_expression.append(r'label check 0')

    # regular_expression.append(r'g_label_pose.valid')
    # regular_expression.append(r'ok ok_vec')
    # regular_expression.append(r'dis_off')
    # regular_expression.append(r'traverse')

    # regular_expression.append(r'fuse label_pose:')
    # regular_expression.append(r'setPoseToLabel')
    regular_expression.append(r'setLabelData')
    # regular_expression.append(r'checkLabelData')
    # regular_expression.append(r'getBestLabel')
    # regular_expression.append(r'erase2')
    # regular_expression.append(r'checkLabelData 1805 near 0')

    # regular_expression.append(r'imu_buildmap_')
    # regular_expression.append(r'test timeout')
    # regular_expression.append(r'autoMode')
    # regular_expression.append(r'test1')
    # regular_expression.append(r'test3')
    # regular_expression.append(r'test4')
    # regular_expression.append(r'test5')
    # regular_expression.append(r'test6')
    # regular_expression.append(r'getGazerLocalPose buf')

    regular_expression.append(r'NO_GAZER_NO_DATA')
    # regular_expression.append(r'g_ekf_ptr=0')
    # regular_expression.append(r'initPose')
    # regular_expression.append(r'PoseEKF(0)')
    # regular_expression.append(r'getGazerRawData false')
    # regular_expression.append(r'gazer pose1')
    # regular_expression.append(r'getGazerRawData buf')
    # regular_expression.append(r'setPoseToLabel')
    # regular_expression.append(r'imu_error!')

    # regular_expression.append(r'insert success:')
    # regular_expression.append(r'setTransForm')

    tool.check_file(src_file, output_path + r"key_log.log", regular_expression)
