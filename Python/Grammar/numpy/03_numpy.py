# 数组运算、切片
import numpy as np

if 1:
    # 数组运算
    arr = np.array([[1., 2., 3.], [4., 5., 6.]])
    arr2 = arr + 1
    print(arr)
    print(arr * arr)
    print(1 / arr)
    print(arr ** 0.5)
    print(arr2 > arr)

if 1:
    # 切片
    arr = np.arange(10)
    print(arr)
    print(arr[5])
    arr[5:8] = 14
    print(arr)
    arr[:] = 0
    print(arr)

if 1:
    # 二维数组
    arr2d = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    print(arr2d[0][2])
    print(arr2d[0, 2])
    # 后2行，前2列
    print(arr2d[1:, :2])

if 1:
    # 布尔型索引
    # 通过布尔型索引选取数组中的数据，将总是创建数据的副本
    names = np.array(['Bob', 'Joe', 'Will', 'Bob', 'Will', 'Joe', 'Joe'])
    data = np.random.randn(7, 4)
    print(data[~(names == "Bob")])
    data[data < 0] = 0
    print(data)

if 1:
    # 花式索引
    arr = np.empty((8, 4))
    for i in range(8):
        arr[i] = i
    print(arr[[4, 3, 0, 6]])

    arr = np.arange(32).reshape((8, 4))
    # 第一个列表一维索引，第二个列表二维索引
    print(arr[[1, 5, 7, 2], [0, 3, 1, 2]])  # [ 4 23 29 10]

