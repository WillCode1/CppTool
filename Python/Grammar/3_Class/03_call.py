# 定制类
# 测试可调用对象和__call__()方法
class Sum:
    def __call__(self, *args, **kwargs):
        res = 0
        for i in range(0, len(args)):
            res = res + args[i]
        print("求和结果：{0}".format(res))
        return res


s = Sum()
s(1, 2, 3, 4, 5)
