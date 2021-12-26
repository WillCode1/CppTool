#!/usr/bin/env python3
# -*- coding: utf-8 -*-

""" Generating random test case """

__author__ = 'Wei Di'

from collections import Iterator
from collections import Iterable
from functools import reduce

list_exam = [1, 2, 3, 4]
tuple_exam = (1, "Will", [2, 4])
dict_exam = {"Beijing": 1, "Shanghai": 2}
set_exam = {1, 2, 3, 2, 1, 2}
set_exam = set(list_exam)

# 列表生成器和生成器
L = [x * x for x in range(10)]
g = (x * x for x in range(10))

# lambda表达式
foo1 = lambda a, b, c: a + b + c
print(foo1)
print(foo1(2, 3, 4))

# 可迭代对象
isinstance([], Iterable)
# 迭代器对象
isinstance((x for x in range(10)), Iterator)
isinstance([], Iterator)
isinstance(iter([]), Iterator)


# 高阶函数=====================================
# map
r = map(int, "12345")
print(r)
print(list(r))
# reduce
res = reduce(lambda x, y: x + y, list(map(int, "12345")))
print(res)


# filter
def is_int(n):
    return isinstance(n, int)


print(list(filter(is_int, [1, "2", 3, "4", 5])))

# sorted
print(sorted(['bob', 'about', 'Zoo', 'Credit'], key=str.lower, reverse=True))
print(sorted([36, 5, -12, 9, -21], key=abs))
