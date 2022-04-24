import struct
import numpy as np
import pptk
import math
import argparse
from tqdm import tqdm


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


def loadfeaturePoint(feature_map, z_min, z_max):
    f = open(feature_map, 'rb')
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

        if pz < z_min or pz  > z_max:
            continue
        x.append(px)
        y.append(py)
        z.append(pz )
        color.append(0.925)

    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    color = np.array(color)
    # point = np.column_stack((z, -x, -y))
    point = np.column_stack((x, y, z))

    return point ,-z 
    # return point, color


def readHpaBinary(filename):
    f = open(filename, 'rb')
    point_num = struct.unpack('q', f.read(8))[0]
    color = []
    point = []

    for i in tqdm(range(point_num),
                  total=point_num,
                  desc='processing points:',
                  ncols=80):

        left_up = (float(struct.unpack('d', f.read(8))[0]),
                   float(struct.unpack('d', f.read(8))[0]))
        right_bottom = (float(struct.unpack('d', f.read(8))[0]),
                        float(struct.unpack('d', f.read(8))[0]))
        prob = struct.unpack('i', f.read(4))[0]
        label = struct.unpack('i', f.read(4))[0]

        if not label == -1:
            point.append([
                0.5 * (left_up[0] + right_bottom[0]),
                0.5 * (left_up[1] + right_bottom[1]), 0
            ])

            if label == 0:  ## parking slot
                color.append(-2.5)
            elif label == 1:  ## dash
                color.append(0.5)
            elif label == 2:  ## lane
                color.append(0.925)
            elif label == 3:  ## arrow
                color.append(-1)
            else:
                color.append(1)

        pass

    print('\rDone')
    print('read {} semantic points\n'.format(len(point)))
    return point, color


def main():

    parser = argparse.ArgumentParser(description='Show Feature PointCloud')
    parser.add_argument('-f',
                        dest='feature',
                        required=True,
                        default='./map.bin',
                        help='feature point-cloud')
    parser.add_argument('-m',
                        dest='map',
                        required=True,
                        help='hpa semantic map ')
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
    feature_point, feature_color = loadfeaturePoint(args.feature, args.z_min,
                                                    args.z_max)

    semantic_point, semantic_color = readHpaBinary(args.map)
    point = np.vstack((feature_point, semantic_point))
    color = np.hstack((feature_color, semantic_color))
    v = pptk.viewer(point)

    # v.attributes(color / 255., x)

    v.attributes(color)

    v.set(point_size=0.03)


if __name__ == '__main__':
    print('usage : python show_feature_pointcloud -m  map_cloud.bin')
    main()
