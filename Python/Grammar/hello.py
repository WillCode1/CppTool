# 定义默认参数要牢记一点：默认参数必须指向不变对象！
# 比较运行结果

def add_end1(L=[]):
    L.append('END')
    return L


def add_end2(L=None):
    if L is None:
        L = []
    L.append('END')
    return L


# 可变参数
def calc(*numbers):
    sum = 0
    for n in numbers:
        sum = sum + n * n
    return sum


nums = [1, 2, 3]
# calc(nums[0], nums[1], nums[2])
calc(*nums)


def person(name, age, **kw):
    print('name:', name, 'age:', age, 'other:', kw)


extra = {'city': 'Beijing', 'job': 'Engineer'}
person('Jack', 24, **extra)

# 列表生成器和生成器
L = [x * x for x in range(10)]
g = (x * x for x in range(10))
