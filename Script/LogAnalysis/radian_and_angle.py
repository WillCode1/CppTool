import math


def angle_normalize(angle, min=-180, max=180):
    while angle < min:
        angle += 360
    while angle >= max:
        angle -= 360
    return angle


def radian2angle(radian):
    return angle_normalize(radian * (180 / math.pi))


def angle2radian(angle):
    return angle * (math.pi / 180)


print(radian2angle(-0.818224))
print(radian2angle(5.457345))

print(radian2angle(2.981638))
print(radian2angle(2.692658))
