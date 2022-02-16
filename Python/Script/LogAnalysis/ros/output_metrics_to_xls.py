import re
import xlwt
import xlrd
import xlutils
from xlutils.copy import copy


def init_excel(sheet):
    sheet.write(1, 21, xlwt.Formula('SUMIF(C8:C300,U2,V8:V300)'))
    sheet.write(1, 22, xlwt.Formula('SUMIF(C8:C300,U2,W8:W300)'))
    sheet.write(1, 23, xlwt.Formula('SUMIF(C8:C300,U2,X8:X300)'))
    sheet.write(1, 24, xlwt.Formula('SUMIF(C8:C300,U2,Y8:Y300)'))
    sheet.write(1, 25, xlwt.Formula('V2/X2'))
    sheet.write(1, 26, xlwt.Formula('W2/Y2'))

    sheet.write(1, 31, xlwt.Formula('SUMIF(C8:C300,U2,AF8:AF300)'))
    sheet.write(1, 32, xlwt.Formula('SUMIF(C8:C300,U2,AG8:AG300)'))
    sheet.write(1, 33, xlwt.Formula('SUMIF(C8:C300,U2,AH8:AH300)'))
    sheet.write(1, 34, xlwt.Formula('SUMIF(C8:C300,U2,AI8:AI300)'))
    sheet.write(1, 35, xlwt.Formula('AF2/AH2'))
    sheet.write(1, 36, xlwt.Formula('AG2/AI2'))
    sheet.write(1, 41, '=MAXIFS(AP8:AP300,C8:C300,U2)')
    sheet.write(1, 42, '=MAXIFS(AQ8:AQ300,C8:C300,U2)')

    sheet.write(2, 21, xlwt.Formula('SUMIF(C8:C300,U3,V8:V300)'))
    sheet.write(2, 22, xlwt.Formula('SUMIF(C8:C300,U3,W8:W300)'))
    sheet.write(2, 23, xlwt.Formula('SUMIF(C8:C300,U3,X8:X300)'))
    sheet.write(2, 24, xlwt.Formula('SUMIF(C8:C300,U3,Y8:Y300)'))
    sheet.write(2, 25, xlwt.Formula('V3/X3'))
    sheet.write(2, 26, xlwt.Formula('W3/Y3'))

    sheet.write(2, 31, xlwt.Formula('SUMIF(C8:C300,U3,AF8:AF300)'))
    sheet.write(2, 32, xlwt.Formula('SUMIF(C8:C300,U3,AG8:AG300)'))
    sheet.write(2, 33, xlwt.Formula('SUMIF(C8:C300,U3,AH8:AH300)'))
    sheet.write(2, 34, xlwt.Formula('SUMIF(C8:C300,U3,AI8:AI300)'))
    sheet.write(2, 35, xlwt.Formula('AF3/AH3'))
    sheet.write(2, 36, xlwt.Formula('AG3/AI3'))
    sheet.write(2, 41, '=MAXIFS(AP8:AP300,C8:C300,U3)')
    sheet.write(2, 42, '=MAXIFS(AQ8:AQ300,C8:C300,U3)')


def write2xls(bm_list, src_name, dst_name):
    workbook = xlrd.open_workbook(src_name)
    # 根据sheet索引或者名称获取sheet内容
    sheet = workbook.sheet_by_index(0)
    # sheet = workbook.sheet_by_name('Sheet1')
    # print (workboot.sheets()[0])

    # sheet的名称，行数，列数
    # print(sheet.name, sheet.nrows, sheet.ncols)

    # 获取整行和整列的值（数组）
    # rows = sheet.row_values(1)  # 获取第2行内容
    bag_names = sheet.col_values(0)  # 获取第1列内容

    for bm in bm_list:
        if bag_names.count(bm.filename) > 1:
            print("Has the same name file!")
        elif bag_names.count(bm.filename) == 1:
            bm.index = bag_names.index(bm.filename)

    workbook = copy(workbook)
    sheet = workbook.get_sheet('Sheet1')
    init_excel(sheet)

    for bm in bm_list:
        sheet.write(bm.index, 21, float(bm.metrics['sf']["total_dis_err"]))
        sheet.write(bm.index, 23, float(bm.metrics['sf']["run_dis"]))
        sheet.write(bm.index, 25, float(bm.metrics['sf']["mean_dis_err"]))
        sheet.write(bm.index, 22, float(bm.metrics['sf']["total_ang_err"]))
        sheet.write(bm.index, 24, float(bm.metrics['sf']["run_ang"]))
        sheet.write(bm.index, 26, float(bm.metrics['sf']["mean_ang_err"]))

        sheet.write(bm.index, 31, float(bm.metrics['ekf']["total_dis_err"]))
        sheet.write(bm.index, 33, float(bm.metrics['ekf']["run_dis"]))
        sheet.write(bm.index, 35, float(bm.metrics['ekf']["mean_dis_err"]))
        sheet.write(bm.index, 32, float(bm.metrics['ekf']["total_ang_err"]))
        sheet.write(bm.index, 34, float(bm.metrics['ekf']["run_ang"]))
        sheet.write(bm.index, 36, float(bm.metrics['ekf']["mean_ang_err"]))

        i = str(bm.index + 1)
        sheet.write(bm.index, 41, xlwt.Formula('IF(AF' + i + '>V' + i + ',AF' + i + '-V' + i + ',0)'))
        sheet.write(bm.index, 42, xlwt.Formula('IF(AG' + i + '>W' + i + ',AG' + i + '-W' + i + ',0)'))

    workbook.save(dst_name)


class BagMetrics:
    # 限制该class实例能添加的属性
    __slots__ = ('index', 'filename', 'metrics', 'track_type')

    def __init__(self, name):
        self.index = None
        self.filename = name
        self.track_type = None
        self.metrics = {
            "sf": {
                "total_dis_err": None,
                "total_ang_err": None,
                "run_dis": None,
                "run_ang": None,
                "mean_dis_err": None,
                "mean_ang_err": None,
                "everytime_dis_err_top_mean": None,
                "everytime_ang_err_top_mean": None,
                "everytime_dis_err_mean": None,
                "everytime_ang_err_mean": None
            },
            "ekf": {
                "total_dis_err": None,
                "total_ang_err": None,
                "run_dis": None,
                "run_ang": None,
                "mean_dis_err": None,
                "mean_ang_err": None,
                "everytime_dis_err_top_mean": None,
                "everytime_ang_err_top_mean": None,
                "everytime_dis_err_mean": None,
                "everytime_ang_err_mean": None
            }
        }


def check_file(file_path):
    old_file = open(file_path, 'r', errors='ignore')

    # 捕获编码异常文件
    try:
        all_line = old_file.readlines()
    except Exception:
        old_file.close()
        return
    old_file.close()

    in_bag = False
    bm = None
    bm_list = []
    for index in range(0, len(all_line)):
        if re.match(r'(\[0m\[ INFO] \[)([0-9]+).([0-9]+)(]: )', all_line[index]) is not None:
            if all_line[index].find("test bag") != -1:
                in_bag = True
                bm = BagMetrics(all_line[index][all_line[index].rfind('/') + 1: all_line[index].rfind('.bag') + 4])
            elif all_line[index].find("one bag end!") != -1:
                in_bag = False
                bm_list.append(bm)
                bm = None
            if in_bag:
                if all_line[index].find("sf metrics:") != -1:
                    bm.track_type = "sf"
                elif all_line[index].find("fuse metrics:") != -1:
                    bm.track_type = "ekf"
                if bm.track_type is not None:
                    if all_line[index].find("total_dis_error:") != -1:
                        bm.metrics[bm.track_type]["total_dis_err"] = all_line[index][
                                                                     all_line[index].find('total_dis_error: ') + 17:
                                                                     all_line[index].find(',')]
                        bm.metrics[bm.track_type]["run_dis"] = all_line[index][
                                                               all_line[index].find('run_dis: ') + 9:all_line[
                                                                   index].index(',', all_line[index].find('run_dis: '))]
                        bm.metrics[bm.track_type]["mean_dis_err"] = all_line[index][
                                                                    all_line[index].find('error_mu: ') + 10:all_line[
                                                                        index].index(',', all_line[index].find(
                                                                        'error_mu: '))]
                    elif all_line[index].find("total_ang_error:") != -1:
                        bm.metrics[bm.track_type]["total_ang_err"] = all_line[index][
                                                                     all_line[index].find('total_ang_error: ') + 17:
                                                                     all_line[index].find(',')]
                        bm.metrics[bm.track_type]["run_ang"] = all_line[index][
                                                               all_line[index].find('run_ang: ') + 9:all_line[
                                                                   index].index(',', all_line[index].find('run_ang: '))]
                        bm.metrics[bm.track_type]["mean_ang_err"] = all_line[index][
                                                                    all_line[index].find('error_mu: ') + 10:all_line[
                                                                        index].index(',', all_line[index].find(
                                                                        'error_mu: '))]
    return bm_list


if __name__ == '__main__':
    bm_list = check_file('/home/will/catkin_ws/log1.txt')
    write2xls(bm_list, '/home/will/Desktop/rosbag_优化场景对比.xls', 'new_res1.xls')
