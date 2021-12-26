# 测试对象的浅拷贝、深拷贝
import copy

class MobliePhone:
    def __init__(self, cpu, screen):
        self.cpu = cpu
        self.screen = screen


class CPU:
    def calculate(self):
        print("cpu对象：", self)


class Screen:
    def show(self):
        print("screen对象：", self)

# 测试变量赋值
c1 = CPU()
s1 = Screen()
c2 = c1
print(c1)
print(c2)

# 测试浅拷贝
print("测试浅拷贝。。。。。。。。")
m1 = MobliePhone(c1, s1)
m2 = copy.copy(m1)
print(m1, m1.cpu, m1.screen)
print(m2, m2.cpu, m2.screen)

# 测试深拷贝
print("测试深拷贝。。。。。。。。")
m3 = MobliePhone(c1, s1)
m4 = copy.deepcopy(m3)
print(m3, m3.cpu, m3.screen)
print(m4, m4.cpu, m4.screen)
