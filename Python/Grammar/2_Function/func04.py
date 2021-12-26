# 测试nonlocal、global关键字的用法
a = 100

def outer():
    b = 10

    def inner():
        nonlocal b
        b = 20
        global a
        a = 1000

    inner()
    print("b: ", b)

outer()
print("a: ", a)
