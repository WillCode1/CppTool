import pptk
import struct
import sys
import numpy as np
import argparse
import os
from tqdm import tqdm


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
                color.append([0.447, 0.623, 0.8117])
            elif label == 1:  ## dash
                color.append([1, 0.58, 0.22])
            elif label == 2:  ## lane
                color.append([0.925, 0, 0.2823])
            elif label == 3:  ## arrow
                color.append([0, 1, 0])
            else:
                color.append([1, 0, 0])
        pass

    print('\rDone')
    print('read {} semantic points\n'.format(len(point)))
    return point, color


def sampleLinePoint(v1, v2, eps=0.01):
    m = np.linalg.norm(v1 - v2, ord=2) / eps
    return np.linspace(v1, v2, int(m), endpoint=False)


def loadTrajectory(filename):
    trajectory = np.genfromtxt(filename, usecols=(1, 2))
    if len(trajectory) == 0:
        return None

    trajectory_line = np.empty([0, 2], dtype=float)
    for (v1, v2) in zip(trajectory[1:], trajectory[2:]):
        trajectory_line = np.vstack((trajectory_line, sampleLinePoint(v1, v2)))

    color = [[252, 212, 54] * len(trajectory_line)]
    color = np.array(color).reshape(len(trajectory_line), -1) / 255.0
    return color, np.c_[trajectory_line, np.zeros(len(trajectory_line))]


def main():

    parser = argparse.ArgumentParser(
        description='Show Hpa Quadtree Occupancy Map')
    parser.add_argument('-m',
                        dest='map',
                        required=True,
                        default='./quadtree',
                        help='quadtree binary map')
    parser.add_argument('-t',
                        dest='trajectory',
                        required=False,
                        help='trajectory of ')

    args = parser.parse_args()
    point, color = readHpaBinary(args.map)

    if args.trajectory is not None and os.path.exists(args.trajectory):
        trajectory_color, trajectory_point = loadTrajectory(args.trajectory)
        point = np.vstack((point, trajectory_point))
        color = np.vstack((color, trajectory_color))
        print('trajectory line is shown as gold line ')

    color = np.array(color)
    point = np.array(point)
    v = pptk.viewer(point)
    v.attributes(color)
    v.set(point_size=0.03)


if __name__ == '__main__':
    main()
