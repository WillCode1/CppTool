# 继承
class Person:
    def __init__(self, name, age):
        self.name = name
        self.__age = age

    def say_age(self):
        print(self.__age)


class Student(Person):
    def __init__(self, name, age, score):
        Person.__init__(self, name, age)
        self.score = score


#Student-->Person-->object类
print(Student.mro())    # 打印类的层次

s = Student("Will", 18, 98)
s.say_age()
print(s.name)
print(s._Person__age)
