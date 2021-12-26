# 列表推导式
a = [x * 2 for x in range(1, 5) if x % 2 == 0]
print(a)

cells = [(row, col) for row in range(1, 10) for col in range(1, 10)]
print(cells)

# 字典推导式
text = "I love you, I love sxt, I love gaoqi"
char_count = {c: text.count(c) for c in text}
print(char_count)

# 集合推导式
my_set = {x for x in range(1, 100) if x % 9 == 0}
print(my_set)

# 生成器推导式
gnt = (x for x in range(4))
print(gnt)
print(tuple(gnt))
print(tuple(gnt))
