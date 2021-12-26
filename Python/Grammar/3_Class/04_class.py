# 测试方法的动态性
# 方法也是对象，可以修改内容（理解为函数指针）
class Person:
    def work(self):
        print("努力工作！")


def play(s):
    print("{0}在玩游戏".format(s))


Person.play = play
p = Person()
p.work()
p.play()

Person.work = play
p.work()
