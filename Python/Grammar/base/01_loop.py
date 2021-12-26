# 遍历list

# 方法1
sum = 0
for x in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
    sum = sum + x
print(sum)

# 方法2
sum = 0
for x in range(101):
    sum = sum + x
print(sum)

# 方法3
seq = ['one', 'two', 'three']
for i, element in enumerate(seq):
    print(i, element)
