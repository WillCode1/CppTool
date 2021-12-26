# 字典序列解包

s = {"name": "will", "age": 18, "job": "master"}

a, b, c = s
e, d, f = s.values()
h, i, j = s.items()

print(a, b, c)
print(e, d, f)
print(h, i, j)
# =======================

# 使用字典表示表格数据

row1 = {"name": "Will", "age": 18, "salary": 20000, "city": "北京"}
row2 = {"name": "Stark", "age": 20, "salary": 30000, "city": "上海"}
row3 = {"name": "Demo", "age": 24, "salary": 50000, "city": "深圳"}

table = [row1, row2, row3]

print(table[1].get("salary"))

# 打印表中所有的薪资
for i in range(len(table)):  # i --> 0,1,2
    print(table[i].get("salary"))

# 打印表的所有数据
for i in range(len(table)):
    print(table[i].get("name"), table[i].get("age"), table[i].get("salary"), table[i].get("city"))
