# 类属性、类方法、静态方法
class Student:
    company = "Willson"  # 类属性
    count = 0

    def __init__(self, name="Dog", age=3):
        self.name = name  # 实例属性
        self.age = age
        Student.count = Student.count + 1

    @classmethod          # 类方法
    def printCompany(cls):
        print(cls.company)

    @staticmethod         # 静态方法
    def add(a, b):
        print("{0} + {1} = {2}".format(a, b, (a+b)))
        return a + b

    def printInfo(self):  # 实例方法
        print("我的公司是：", Student.company)
        print("{0}的年龄是：{1}".format(self.name, self.age))


s1 = Student("Will", 18)
s2 = Student()
s3 = Student()

s1.printInfo()
print("一共创建{0}个Student对象".format(Student.count))

Student.printCompany()
Student.add(20, 30)
