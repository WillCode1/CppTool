def printMax(a, b):
    """用于打印较大值"""
    if a > b:
        print(a)
    else:
        print(b)


printMax(10, 20)
printMax(100, 200)

help(printMax.__doc__)
