# 数组生成、类型转换
import numpy as np
import time

# ndarray是一个通用的同构数据多维容器
# numpy数组和list比较
if 0:
    my_arr = np.arange(1000000)
    my_list = list(range(1000000))

    start = time.time()
    for _ in range(10): my_arr = my_arr * 2
    end = time.time()
    print('Running time: %s Seconds' % (end - start))

    start = time.time()
    for _ in range(10): my_list = [x * 2 for x in my_list]
    end = time.time()
    print('Running time: %s Seconds' % (end - start))

if 1:
    # numpy随机生成多维数组
    data = np.random.randn(2, 3)
    print(data)
    print(data.shape)
    print(data.dtype)

if 1:
    # array函数生成ndarray
    arr = np.array([1, 2, 3, 4, 5])
    arr = np.asarray([[1, 2], [3, 4]])
    print(arr)
    print(arr.ndim, arr.shape, arr.dtype)

if 1:
    # 传递元组
    arr0 = np.zeros(10)
    arr1 = np.ones((3, 6))
    arr2 = np.empty((2, 3, 2), dtype=np.int32)
    arr3 = np.full((2, 2), 1.)
    matrix = np.eye(3)  # 3维方阵
    print(arr0)
    print(arr1)
    print(arr2)
    print(arr3)
    print(matrix)

if 1:
    # 类型转换
    arr = np.array(['1', '2', '3', '4', '5'])
    print(arr.dtype)

    float_arr = arr.astype(np.float64)
    print(float_arr)
    print(float_arr.dtype)

