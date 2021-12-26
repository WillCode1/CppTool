# 调试技巧
import sys
import logging
# logging.basicConfig(level=logging.INFO)
logging.basicConfig(level=logging.ERROR)


# 打印行号
def get_cur_info():
    print(str(sys._getframe().f_code.co_filename))  # 当前文件名
    print(str(sys._getframe(0).f_code.co_name))     # 当前函数名
    print(str(sys._getframe(1).f_code.co_name))     # 调用该函数的函数名字，如果没有被调用，返回module
    print("here is at line " + str(sys._getframe().f_lineno))# 当前行号


# assert断言
# 启动Python解释器时可以用-O参数来关闭assert
# python -O xxx.py
def foo1(s):
    n = int(s)
    assert n != 0, "n is zero!"
    logging.info('n = %d' % n)
    return 10/n


# logging
def foo2(s):
    n = int(s)
    logging.info('n = %d' % n)
    logging.debug('n = %d' % n)
    return 10/n


if __name__ == '__main__':
    # get_cur_info()
    print(foo2('0'))

