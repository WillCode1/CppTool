import numpy as np
import math
import matplotlib.pyplot as plt
'''
x: State Vector [x y yaw v]
u: [v, yawrate]
这是一种基于扩展卡尔曼滤波(EKF)的传感器融合定位。
蓝线是真实轨迹，黑线是船位推算轨迹，
绿点为定位观测(如GPS)，红线为EKF估计轨迹。
红色椭圆用EKF估计协方差椭圆。
'''
DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]

show_animation = True


def get_control(rand_yaw=False):
    v = 1.0  # [m/s]
    if rand_yaw:
        yawrate = np.random.randn()
    else:
        yawrate = 0.1  # [rad/s]
    u = np.matrix([v, yawrate]).T
    return u


def observation(xTrue, xd, u):
    xTrue = system_model(xTrue, u)

    #  Simulation parameter
    Qsim = np.diag([0.5, 0.5]) ** 2
    Rsim = np.diag([1.0, np.deg2rad(30.0)]) ** 2

    # add noise to gps x-y，可以加高斯噪声
    zx = xTrue[0, 0] + np.random.randn() * Qsim[0, 0]
    zy = xTrue[1, 0] + np.random.randn() * Qsim[1, 1]
    z = np.matrix([zx, zy]).T

    # add noise to input，可以加高斯噪声
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.matrix([ud1, ud2]).T

    xd = system_model(xd, ud)

    return xTrue, z, xd, ud


# 系统模型g(x, u)，用于预测，非线性模型
def system_model(x, u):
    F = np.matrix([[1.0, 0, 0, 0],
                   [0, 1.0, 0, 0],
                   [0, 0, 1.0, 0],
                   [0, 0, 0, 0]])

    # x = x + vt * cos(yaw)
    # y = y + vt * sin(yaw)
    # yaw = yaw + yawrate * t
    # v = v
    B = np.matrix([[DT * math.cos(x[2, 0]), 0],
                   [DT * math.sin(x[2, 0]), 0],
                   [0.0, DT],
                   [1.0, 0.0]])

    # 非线性变换依然对应矩阵，只不过这个矩阵不是常矩阵了，而是函数矩阵
    x = F * x + B * u

    return x


# 观测模型h(x)，这里是线性模型，也可以改为非线性模型
def observation_model(x):
    H = jacobH(x)

    z = H * x

    return z


# F为状态转移矩阵，表示将t−1时刻的状态向量转移至t时刻的状态向量
# 预测模型的jacobian矩阵
def jacobF(x, u):
    # Jacobian of Motion Model
    """
    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.matrix([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


# H为转换矩阵，状态空间映射到测量空间
def jacobH(x):
    # Jacobian of Observation Model
    jH = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u):
    # Estimation parameter of EKF
    # Q: 预测模型的协方差
    # R: 观测模型的协方差
    Q = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0]) ** 2
    R = np.diag([10.0, np.deg2rad(400.0)]) ** 2

    #  Predict
    xPred = system_model(xEst, u)
    jF = jacobF(xPred, u)
    PPred = jF * PEst * jF.T + Q

    #  Update
    jH = jacobH(xPred)
    K = PPred * jH.T * np.linalg.inv(jH * PPred * jH.T + R)
    zPred = observation_model(xPred)
    xEst = xPred + K * (z - zPred)
    PEst = (np.eye(len(xEst)) - K * jH) * PPred

    if xPred[0] < xEst[0] < z[0] or z[0] < xEst[0] < xPred[0]:
        pass
    else:
        print(xEst[0])
        print(xPred[0])
        print(z[0])
        print(1111111)

    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.matrix([[math.cos(angle), math.sin(angle)],
                   [-math.sin(angle), math.cos(angle)]])
    fx = R * np.matrix([x, y])
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    print(__file__ + " start!!")

    time = 0.0

    # State Vector [x y yaw v]
    xEst = np.matrix(np.zeros((4, 1)))
    xTrue = np.matrix(np.zeros((4, 1)))
    PEst = np.eye(4)

    xDR = np.matrix(np.zeros((4, 1)))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((1, 2))

    while SIM_TIME >= time:
        time += DT
        u = get_control(rand_yaw=False)

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.vstack((hz, z.T))

        if show_animation:
            plt.cla()
            # 绿点为定位观测(如GPS)
            plt.plot(hz[:, 0], hz[:, 1], ".g")
            # 蓝线是真实轨迹
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            # 黑线是船位推算轨迹
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k")
            # 红线为EKF估计轨迹
            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r")
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
