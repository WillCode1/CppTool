# 测试私有属性和私有方法
class Employee:
    __company = "WIS"

    def __init__(self, name, age):
        self.name = name
        self.__age = age    # __member，表示私有(实例)属性

    def __work(self):       # 私有方法
        print("年龄：{0}".format(self.__age))
        print("好好工作！")


e = Employee("Will", 18)

print(e.name)
# print(e.age)

# _ClassName__member，可以访问私有属性
print(e._Employee__age)
e._Employee__work()
# print(dir(e))
print(Employee._Employee__company)
