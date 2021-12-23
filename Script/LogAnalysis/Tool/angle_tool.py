import math


def check_in_period(angle: float, min: float, max: float, period: float):
    while angle < min:
        angle += period
    while angle >= max:
        angle -= period
    return angle


def angle_normalize(angle, min=-180, max=180):
    return check_in_period(angle, min, max, 360)


def radian_normalize(radian, min=-math.pi, max=math.pi):
    return check_in_period(radian, min, max, 2 * math.pi)


def radian2angle(radian):
    return radian * (180 / math.pi)


def angle2radian(angle):
    return angle * (math.pi / 180)


if __name__ == '__main__':
    print(radian2angle(-0.818224))
    print(radian2angle(5.457345))

    print(radian2angle(2.981638))
    print(radian2angle(2.692658))
