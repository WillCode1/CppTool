# 测试object
# 重写__str__()


class A:
    def __init__(self, name):
        self.name = name

    def __str__(self):
        return "名字是：{0}".format(self.name)


obj = object()
s = A("Will")
print(dir(obj))
print(dir(s))

print(s)
