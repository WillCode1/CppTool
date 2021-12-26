# 过滤用于仿真的数据
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
            if re.match(value, line):
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

        if not os.path.exists(file_path):
            os.makedirs(file_path)

        content = ''.join(filter_res)
        write_file = open(dest_file, 'w', encoding='utf-8')
        write_file.write(content)
        write_file.close()


if __name__ == '__main__':
    # output_path = r"/work/robot/Bin/Arm-Release/"
    # src_file = r"/work/robot/Bin/Arm-Release/" + r"log"

    src_file = ct.get_desktop_path() + r"UT_log\\log_ut"
    output_path = ct.get_desktop_path() + r"UT_log\\filter_log.txt"

    tool = FilterLog()

    # regular_expression = [r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} msg yaw: ',
    #                       r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} EncodeData: ',
    #                       r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} setLabelData 748 pose ']

    # msg yaw: 2.88363, time:1287864cur:1286065, syst:[2021-09-15 Wed 16:55:00 004] thread_a:time_cur:1285625  time_last:1285540
    # msg left: -98410, time:1197435cur:1195649, syst:1195241
    regular_expression = [r'msg yaw: ',
                          r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} setEncodeData ',
                          r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} setLabelData 766 pose ']

    tool.check_file(src_file, output_path, regular_expression)
    # regular_expression = [r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} (fuse|forcast) cur_robot_pose:']
    # regular_expression = [r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} insert 3 sensordata:']
