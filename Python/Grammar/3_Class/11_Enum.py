# 枚举类
from enum import Enum, unique

Weekday = Enum('Weekday', ('Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'))

for name, member in Weekday.__members__.items():
    print(name, member, member.value)


# @unique装饰器可以帮助我们检查保证没有重复值
@unique
class Weekday(Enum):
    Sun = 0
    Mon = 1
    Tue = 2
    Wed = 3
    Thu = 4
    Fri = 5
    Sat = 6
