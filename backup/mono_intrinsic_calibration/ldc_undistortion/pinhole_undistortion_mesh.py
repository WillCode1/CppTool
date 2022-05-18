import os, sys, cv2
import numpy as np
import matplotlib.pyplot as plt
import math


def FillMiddle(ad_offset, offset, height, width):
    ad_offset[int(height / 2), :int(width / 2)] = (offset[int(height / 2), :int(width / 2)] + \
                                                   offset[int(height / 2) - 1, :int(width / 2)]) / 2.0
    ad_offset[int(height / 2), int(width / 2) + 1:] = (offset[int(height / 2), int(width / 2):] + \
                                                       offset[int(height / 2) - 1, int(width / 2):]) / 2.0
    ad_offset[:int(height / 2), int(width / 2)] = (offset[:int(height / 2), int(width / 2)] + \
                                                   offset[:int(height / 2), int(width / 2) - 1]) / 2.0
    ad_offset[int(height / 2) + 1:, int(width / 2)] = (offset[int(height / 2):, int(width / 2)] + \
                                                       offset[int(height / 2):, int(width / 2) - 1]) / 2.0
    ad_offset[int(height / 2), int(width / 2)] = (ad_offset[int(height / 2) - 1, int(width / 2)] + \
                                                  ad_offset[int(height / 2) + 1, int(width / 2)] + \
                                                  ad_offset[int(height / 2), int(width / 2) - 1] + \
                                                  ad_offset[int(height / 2), int(width / 2) + 1]) / 4.0


def main():
    m = 4
    m_step = 2 ** m

    if len(sys.argv) == 4:
        img_file_name = str(sys.argv[1])
        intrinsic_file_name = str(sys.argv[2])
        output_name = str(sys.argv[3])
    else:
        print('input not correct!!')
        exit(-1)

    img_ori = cv2.imread(img_file_name)

    height, width = img_ori.shape[:2]

    intrinsic_file = intrinsic_file_name
    if os.path.isfile(intrinsic_file):
        fp = open(intrinsic_file, 'r')
        param_list = [l.split() for l in fp.readlines()]
        fx = float(param_list[0][1])
        fy = float(param_list[1][1])
        cx = float(param_list[2][1])
        cy = float(param_list[3][1])

        camera_matrix = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0, 0, 1]], dtype=np.float32)

        # fx_out, fy_out, cx_out, cy_out can be adjusted 
        # according to the specific situation
        fx_out = fx 
        fy_out = fy
        cx_out = cx
        cy_out = cy
	
        camera_matrix_out = np.array([[fx_out, 0, cx_out],
                                      [0, fy_out, cy_out],
                                      [0, 0, 1]], dtype=np.float32)

        distortion = np.zeros(5)
        # load radial distortion
        if len(param_list[4]) == 4:
            for i in range(1, len(param_list[4]) - 1):
                distortion[i - 1] = param_list[4][i]
            distortion[-1] = param_list[4][3]
        else:
            for i in range(1, len(param_list[4])):
                distortion[i - 1] = param_list[4][i]
        # load tangential distortion
        for i in range(1, len(param_list[5])):
            distortion[i + 1] = param_list[5][i]

        rotation = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], dtype=np.float32)
        fp.close()

    map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, distortion, rotation,
                                             camera_matrix_out, (width, height), cv2.CV_32FC1)

    img = cv2.remap(img_ori, map1, map2, interpolation=cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)

    plt.imshow(img)
    plt.show()

    with open(output_name, 'w+') as fp:
        offset_x = np.zeros(map1.shape)
        offset_y = np.zeros(map2.shape)
        for i in range(height):
            for j in range(width):
                offset_x[i, j] = (map1[i, j] - j) * 8
                offset_y[i, j] = (map2[i, j] - i) * 8

        ad_offset_x = np.zeros((map1.shape[0] + 1, map1.shape[1] + 1))
        ad_offset_y = np.zeros((map2.shape[0] + 1, map2.shape[1] + 1))
        # fill adjust offset
        # 1st Quadrant
        ad_offset_x[:int(height / 2), int(width / 2) + 1:] = offset_x[:int(height / 2), int(width / 2):]
        ad_offset_y[:int(height / 2), int(width / 2) + 1:] = offset_y[:int(height / 2), int(width / 2):]

        # 2nd Quadrant
        ad_offset_x[:int(height / 2), :int(width / 2)] = offset_x[:int(height / 2), :int(width / 2)]
        ad_offset_y[:int(height / 2), :int(width / 2)] = offset_y[:int(height / 2), :int(width / 2)]

        # 3rd Quadrant
        ad_offset_x[int(height / 2) + 1:, :int(width / 2)] = offset_x[int(height / 2):, :int(width / 2)]
        ad_offset_y[int(height / 2) + 1:, :int(width / 2)] = offset_y[int(height / 2):, :int(width / 2)]

        # 4th Quadrant
        ad_offset_x[int(height / 2) + 1:, int(width / 2) + 1:] = offset_x[int(height / 2):, int(width / 2):]
        ad_offset_y[int(height / 2) + 1:, int(width / 2) + 1:] = offset_y[int(height / 2):, int(width / 2):]

        FillMiddle(ad_offset_x, offset_x, height, width)
        FillMiddle(ad_offset_y, offset_y, height, width)

        for i in range(0, height + 1, m_step):
            for j in range(0, width + 1, m_step):
                fp.write(str(int(round(ad_offset_x[i, j]))) + ' ' + str(int(round(ad_offset_y[i, j]))) + '\n')
    fp.close()
    print("done")


main()
