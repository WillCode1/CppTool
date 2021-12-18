import numpy as np
import re
import os


def get_desktop_path() -> str:
    return os.path.join(os.path.expanduser("~"), 'Desktop\\')


def split_file_path_and_name(filepath: str) -> (str, str):
    (file_path, file_name) = os.path.split(filepath)
    return file_path, file_name


def get_file_name_without_extension(file_name: str) -> str:
    (file_name_without_extension, extension) = os.path.splitext(file_name)
    return file_name_without_extension


def match_extension(file_name: str, file_type: list) -> bool:
    # (filepath, tempfilename) = os.path.split(file_name)
    (filename, extension) = os.path.splitext(file_name)
    return extension in file_type


# https://www.cnblogs.com/lingxia/p/7682301.html
def is_float(s):
    s = str(s)
    if s.count('.') == 1:
        left = s.split('.')[0]
        right = s.split('.')[1]
        if not right.isdigit():
            return False

        if left.count('-') == 1 and left.startswith('-'):
            num = left.split('-')[-1]
            if num.isdigit():
                return True
        elif left.isdigit():
            return True
    return False


# https://www.runoob.com/python3/python3-check-is-number.html
def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        pass

    try:
        import unicodedata
        unicodedata.numeric(s)
        return True
    except (TypeError, ValueError):
        pass

    return False


def filter_number(table):
    res = []
    for i in table:
        if is_number(i):
            res.append(i)
    return res


def filter_yaw(table):
    res = []
    for i in table:
        if is_number(i):
            res.append(i)
        elif i.find('syst:') > -1:
            res.append(i[i.find(':')+1:])
    return res


# https://www.cnblogs.com/picassooo/p/11964411.html
# data = np.loadtxt('pose.txt', dtype=np.float32, delimiter=' ')


# txt2matrix
# 数据文件转矩阵
# path: 数据文件路径
# delimiter: 行内字段分隔符
def loadtxt2matrix(path, delimiter=' ', data_type=np.float32, filte_rule=filter_number):
    file = open(path, 'r', encoding='utf-8')
    # string = file.read()
    # all_lines = string.splitlines()  # splitlines默认参数是‘\n’
    all_lines = file.readlines()
    file.close()
    data = [[i for i in filte_rule(line.strip().split(delimiter))] for line in all_lines]
    return np.array(data, dtype=data_type)
