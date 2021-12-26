# 实例属性、实例方法、dir()、__dict__()、isinstance()
class Student:
    # 限制该class实例能添加的属性
    # 使用__slots__要注意，__slots__定义的属性仅对当前类实例起作用，对继承的子类是不起作用的
    __slots__ = ('name', 'age')

    def __init__(self, name, age):
        self.name = name
        self.age = age

    def printInfo(self):
        print("{0}的年龄是：{1}".format(self.name, self.age))


s1 = Student("Will", 18)
s1.score = 100  # 实例添加的额外属性

print(s1.score)

s1.printInfo()
Student.printInfo(s1)
# ===================================

print(dir(s1))                  # 获得对象的所有属性、方法
print(s1.__dict__)              # 获得对象的属性字典
print(isinstance(s1, Student))  # 判断对象是不是指定类型
