# 可变参数

def foo1(a, b, *c, **d):
    print(a, b, c, d)


def foo2(*a, b, c):
    print(a, b, c)


foo1(8, 9, 20, 30, name='gaoqi', age='18')
foo2(8, 9, b=20, c=30)