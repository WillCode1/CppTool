# list倒序
l = [1, 2, 3, 4]
print(l[-1])

# tuple
t = ('a', 'b', ['A', 'B'])
t[2][0] = 'X'
t[2][1] = 'Y'
print(t)

# dict
d = {'Michael': 95, 'Bob': 75, 'Tracy': 85}
print(d['Bob'])
print(d.pop('Michael'))
print(d.get('Michael', -1))

# set
s1 = set([1, 2, 3, 3, 2, 4])
print(s1)
s1.remove(4)
s1.add(4)

s2 = {3, 4, 5}
print(s1 & s2)
