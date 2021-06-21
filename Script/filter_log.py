import os
import re

def get_desktop_path() -> str:
    return os.path.join(os.path.expanduser("~"), 'Desktop\\')

def fetch_file_name_by_path(filepath: str) -> str:
    (file_path, file_name) = os.path.split(filepath)
    return file_name


class FilterLog:
    def __init__(self):
        self.file_name = None

    def check_line(self, line: str, regular_expression: list) -> bool:
        for index, value in enumerate(regular_expression):
            if re.match(value, line):
                return True

    def check_file(self, src_file: str, dest_file: str, regular_expression: list):
        self.file_name = fetch_file_name_by_path(src_file)
        read_file = None
        try:
            read_file = open(src_file, 'r', errors='ignore', encoding='utf-8')
            all_lines = read_file.readlines()
        except FileNotFoundError:
            print("{0}: doesn't exist!".format(self.file_name))
            return
        except UnicodeDecodeError:
            read_file.close()
            print("{0}: read fail!".format(self.file_name))
            return

        filter_res = []
        for index, value in enumerate(all_lines):
            if self.check_line(value, regular_expression):
                filter_res.append(value)

        content = ''.join(filter_res)
        write_file = open(dest_file, 'w', encoding='utf-8')
        write_file.write(content)
        write_file.close()


if __name__ == '__main__':
    output_path = r"D:\\workspace\\log\\"

    src_file = get_desktop_path() + r"log_duanwu"
    pose_file = output_path + r"pose.txt"
    sensordata_file = output_path + r"sensordata.txt"

    tool = FilterLog()

    regular_expression = [r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} fuse cur_robot_pose:',
                          r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} forcast cur_robot_pose:']
    tool.check_file(src_file, pose_file, regular_expression)

    regular_expression = [r'(\[\d{4}-\d{2}-\d{2} [a-zA-Z]+ \d{2}\:\d{2}\:\d{2} \d+\]){1} insert 3 sensordata:']
    tool.check_file(src_file, sensordata_file, regular_expression)
