# 测试zip()并迭代

names = ("Will", "Stark", "Demo")
ages = (18, 29, 43)
jobs = ("A", "B", "C")

for name, age, job in zip(names, ages, jobs):
    print("{0}--{1}--{2}".format(name, age, job))

for i in range(3):
    print("{0}--{1}--{2}".format(names[i], ages[i], jobs[i]))
