import struct
import numpy as np
import pptk
import math
import argparse


def compass_to_rgb(h, s=1, v=1):
    h = float(h)
    s = float(s)
    v = float(v)
    h60 = h / 60.0
    h60f = math.floor(h60)
    hi = int(h60f) % 6
    f = h60 - h60f
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    r, g, b = 0, 0, 0
    if hi == 0: r, g, b = v, t, p
    elif hi == 1: r, g, b = q, v, p
    elif hi == 2: r, g, b = p, v, t
    elif hi == 3: r, g, b = p, q, v
    elif hi == 4: r, g, b = t, p, v
    elif hi == 5: r, g, b = v, p, q
    r, g, b = int(r * 255), int(g * 255), int(b * 255)
    return [r, g, b]


def main():

    parser = argparse.ArgumentParser(description='Show Feature PointCloud')
    parser.add_argument('-m',
                        dest='map',
                        required=True,
                        default='./map.bin',
                        help='feature point-cloud')
    parser.add_argument('-max',
                        dest='z_max',
                        required=False,
                        default=3 ,
                        help='max value of z')
    parser.add_argument('-min',
                        dest='z_min',
                        required=False,
                        default=-0.2,
                        help='min value of z')

    args = parser.parse_args()

    f = open(args.map, 'rb')
    point_num = struct.unpack('Q', f.read(8))[0]
    print('point num : ', point_num)

    x = []
    y = []
    z = []
    color = []

    for i in range(point_num):
        id = struct.unpack('Q', f.read(8))[0]
        px = float(struct.unpack('f', f.read(4))[0])
        py = float(struct.unpack('f', f.read(4))[0])
        pz = float(struct.unpack('f', f.read(4))[0])

        if pz < args.z_min or pz > args.z_max:
            continue
        x.append(px)
        y.append(py)
        z.append(pz)
        color.append(0.925)

    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    color = np.array(color)
    point = np.column_stack((x,y, z))
    v = pptk.viewer(point)

    v.attributes(-z)

    v.set(point_size=0.03)


if __name__ == '__main__':
    print('usage : python show_feature_pointcloud -m  map_cloud.bin')
    main()
