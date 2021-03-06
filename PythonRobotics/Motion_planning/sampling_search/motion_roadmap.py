# coding=utf-8
import cv2
import numpy as np
import Motion_planning.sampling_search.motion_planning_tool as mpt
import math


class MotionRoadmap(object):
    def __init__(self, map_img, start=None, goal=None):
        ## 初始化实例，需要输入一张 bmp 格式的地图
        self.map = map_img
        size = self.map.shape
        self.point_start = np.mat([0, 0])
        self.point_goal = np.mat([size[0] - 1, size[1] - 1])

    def prm_planning(self, num_sample=100, distance_neighbor=200, draw=True):
        ''' 概率路线图算法（PRM算法）。

        本函数可对 self.map 进行 PRM 规划并绘制图像。

        Args:
            num_sample: 采样点个数，默认100。int
            distance_neighbor: 邻域距离，默认100。float
            draw: 绘制图像指令，True表示绘制。bool
        Return:
            本函数没有返回值，但会根据计算结果赋值（或定义）以下属性变量：
                self.vertex: 顶点集，随机采样所生成的顶点。numpy.mat
                self.adjacency_mat: 邻接矩阵，定义顶点之间的连接性。numpy.mat
                self.close_list: 闭集，A*算法所得到的闭集。numpy.mat
                                 数据含义：[[节点索引，父节点索引，路径总代价]]
        Raises:
            暂无明显的接口错误
        Example:
            mr = MotionRoadmap(img)
            mr.prm_planning(num_sample=50, distance_neighbor=150, draw=True)
        '''
        print('开始 PRM 路径规划，请等待...')
        # 地图灰度化
        image_gray = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)
        # 地图二值化
        ret, img_binary = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY)
        # 初始化顶点集
        vertex = np.vstack((self.point_start, self.point_goal))
        ## 构造地图
        # 采样并添加顶点
        while vertex.shape[0] < (num_sample + 2):
            # 随机采样
            x = np.mat(np.random.randint(0, img_binary.shape[0] - 1, (1, 2)))
            # 点碰撞检测，将合理点添加到顶点集
            if mpt.check_point(x[0, :], img_binary):
                vertex = np.vstack((vertex, x))
        ## 构造邻接矩阵
        adjacency_mat = np.zeros((num_sample + 2, num_sample + 2))
        for i in range(num_sample + 2):
            for j in range(num_sample + 2):
                # 如果距离小于 distance_neighbor 且路径不碰撞
                if ((mpt.straight_distance(vertex[i, :], vertex[j, :]) <= distance_neighbor) and
                        (mpt.check_path(vertex[i, :], vertex[j, :], img_binary))):
                    adjacency_mat[i, j] = 1  # 邻接矩阵置为1
        ## A*算法搜索最佳路径
        self.vertex, self.adjacency_mat, self.close_list, find = mpt.A_star_algorithm(vertex, adjacency_mat, 0, 1)
        ## 根据关键字确定是否绘图
        if draw:
            if find:
                mpt.A_star_plot(self.map, self.vertex, self.adjacency_mat, self.close_list)
            else:
                print('没有找到解，无法绘图！')

    def rrt_planning(self, step_size=20, threshold=20, limit_try=20000, draw=True):
        ''' 快速扩展随机树算法（RRT算法）。

        本函数可对 self.map 进行 RRT 规划并绘制图像。

        Args:
            step_size: 搜索步长，默认20。int
            threshold: 判断阈值，小于此值将被视作同一个点，不可大于 step_size，默认20。float
            limit_try: 尝试次数。默认20000。int
            draw: 绘制图像指令，缺省表示绘制，’None‘表示不绘制。string
        Return:
            本函数没有返回值，但会根据计算结果赋值（或定义）以下属性变量：
                self.rrt_tree: 所生成的rrt树。numpy.mat
                    数据含义: [[横坐标, 纵坐标, 父节点索引]]，其中第一个点为根，最后一个点为终点梢

        Raises:
            暂无明显的接口错误
        Example:
            mr = MotionRoadmap(img)
            mr.rrt_planning(step_size=25, threshold=30, limit_try=15000, draw='None')

        '''
        print('开始 RRT 路径规划，请等待...')
        # 地图灰度化
        image_gray = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)
        # 地图二值化
        ret, img_binary = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY)
        # 初始化 RRT 树:[横坐标，纵坐标，父节点索引]
        rrt_tree = np.hstack((self.point_start, [[0]]))
        # 初始化尝试次数
        num_try = 0
        # 成功标识
        path_found = False

        while num_try <= limit_try:
            ## 随机生长采样点
            if np.random.rand() < 0.5:
                # 在地图范围内随机采样一个像素
                sample = np.mat(np.random.randint(0, img_binary.shape[0] - 1, (1, 2)))
            else:
                sample = self.point_goal

            ## 选择rrt树中离采样点最近的点
            # 计算各点与采样点的距离
            mat_distance = mpt.straight_distance(rrt_tree[:, 0: 2], sample)
            # 距离最小点的索引
            index_closest = np.argmin(mat_distance, 0)[0, 0]  # 末尾索引用以取数值，否则为矩阵
            point_closest = rrt_tree[index_closest, 0: 2]

            ## 从距离最小点向采样点移动 step_size 距离，并进行碰撞检测
            theta_dir = math.atan2(sample[0, 0] - point_closest[0, 0], sample[0, 1] - point_closest[0, 1])
            point_move = point_closest + step_size * np.mat([math.sin(theta_dir), math.cos(theta_dir)])
            # 将坐标化为整数
            point_move = np.around(point_move).astype(int)
            if not mpt.check_path(point_closest, point_move, img_binary):
                num_try = num_try + 1
                continue

            ## 成功检测
            if mpt.straight_distance(point_move, self.point_goal) < threshold:
                path_found = True
                # 加入到rrt树
                point_move = np.hstack((point_move, [[index_closest]]))
                rrt_tree = np.vstack((rrt_tree, point_move))
                break

            ## 计算rrt树中各点与新点的距离，如果均大于 threshold 的，则添加新点到rrt树
            mat_distance = mpt.straight_distance(rrt_tree[:, 0: 2], point_move)
            if np.min(mat_distance, 0) < threshold:
                num_try = num_try + 1
                continue
            # 为新点加入父节点索引
            point_move = np.hstack((point_move, [[index_closest]]))
            rrt_tree = np.vstack((rrt_tree, point_move))

        if path_found:
            print('规划成功！')
            ## 根据关键字确定是否绘图
            if draw:
                mpt.tree_plot(self.map, rrt_tree)
        else:
            print('没有找到解。')

    def pf_planning(self, k_a=50.0, k_r=50.0, d_o=3.0, limit_try=1000, draw=True):
        ''' 人工势场法（potential field planning 算法）。

        本函数可对 self.map 进行人工势场法规划并绘制图像。

        Args:
            k_a: 引力增益，默认20。int
            k_r: 斥力增益，默认20.0。float
            d_o: 障碍物的作用范围，默认30.0。float
            draw: 绘制图像指令，缺省表示绘制，’None‘表示不绘制。string
        Return:
            本函数没有返回值，但会根据计算结果赋值（或定义）以下属性变量：
                self.img_potential: 势场图数据。numpy.mat
                self.point_road: 路径点数据。numpy.mat
        Raises:
            暂无明显的接口错误
        Example:
            mr = MotionRoadmap(img)
            mr.pf_planning(a=25, r=30, p='None')

        '''
        print('开始人工势场法路径规划，请等待...')
        ## 预处理
        # 地图灰度化
        image_gray = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)
        # 地图二值化
        ret, img_binary = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY)
        # 创建障碍物坐标集
        mat_img_binary = np.mat(img_binary)
        postion_obs = np.argwhere(mat_img_binary == [0])
        # 成功标识
        path_found = True

        ## 创建势场图
        img_potential = np.mat(img_binary) * 0.0
        for x in range(img_potential.shape[0]):
            for y in range(img_potential.shape[1]):
                # 目标点的引力场 ug
                ug = k_a * mpt.straight_distance(np.mat([x, y]), self.point_goal)[0, 0]
                # 障碍物的斥力场 uo
                # 最近障碍物的距离
                d_min = np.min(mpt.straight_distance(postion_obs, np.mat([x, y])), 0)
                if d_min < d_o:
                    if d_min <= 0.5:
                        d_min = 0.1  # 所有障碍物内部点都作为障碍物边缘点处理
                    uo = k_r * (1.0 / d_min - 1.0 / d_o) ** 2 * \
                         mpt.straight_distance(np.mat([x, y]), self.point_goal)[0, 0] ** 0.25
                else:
                    # 如果障碍物距离大于 d_o ,则不发生斥力作用
                    uo = 0.0
                img_potential[x, y] = ug + uo
        print("势场图创建完毕")
        # self.img_potential = img_potential
        point_current = self.point_start
        potential_current = img_potential[point_current[0, 0], point_current[0, 1]]
        motion_direction = np.mat([[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]])
        point_road = point_current
        num_try = 0
        while mpt.straight_distance(point_current, self.point_goal)[0, 0] > 0.0:
            if num_try < limit_try:
                # potential_temp = potential_current
                point_temp = point_current
                for d in motion_direction:
                    # local_tag = True
                    index_x = point_temp[0, 0] + d[0, 0]
                    index_y = point_temp[0, 1] + d[0, 1]
                    if ((index_x < 0) or (index_x >= img_potential.shape[0]) or
                            (index_y < 0) or (index_y >= img_potential.shape[1])):
                        potential_next = float('inf')
                    else:
                        potential_next = img_potential[index_x, index_y]
                    if potential_current > potential_next:
                        potential_current = potential_next
                        point_current = np.mat([index_x, index_y])

                point_road = np.vstack((point_road, point_current))
                num_try = num_try + 1
            else:
                path_found = False
                break
        self.point_road = point_road
        ## 根据关键字确定是否绘图
        if draw:
            if path_found:
                print('找到解！')
                mpt.potential_plot(self.map, img_potential, point_road)
            else:
                print('陷入局部最优，没有找到解，无法绘图。')
                mpt.potential_plot(self.map, img_potential, point_road)


if __name__ == "__main__":
    ## 预处理
    # 图像路径
    image_path = "./map/map_1.bmp"
    # 读取图像
    img = cv2.imread(image_path)  # np.ndarray RGB uint8
    img = cv2.resize(img, (500, 500))

    start = np.mat([0, 99])
    goal = np.mat([95, 5])
    mr = MotionRoadmap(img, start, goal)

    # mr.prm_planning(100, 100)
    mr.rrt_planning(20, 20, 15000)
    # mr.pf_planning()
