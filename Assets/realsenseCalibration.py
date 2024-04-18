import numpy as np
import sys
from scipy.optimize import least_squares

# python .\realsenseCalibration.py 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 1,1,1,1,1,1 1,1,1,1,1,1 1,0,0,4,0,1,0,8,0,0,1,12
if len(sys.argv) != 6:
    print("param error")

else:
    # 接收命令行参数
    list_as_tf1 = sys.argv[1]
    list_as_tf2 = sys.argv[2]
    list_as_p1 = sys.argv[3]
    list_as_p2 = sys.argv[4]
    list_as_init = sys.argv[5]

    # TF树 EndPoint转到机械臂Base
    # TF_1 = np.array([[11, 12, 13, 14],
    #                  [21, 22, 23, 24],
    #                  [31, 32, 33, 34],
    #                  [0,  0,  0,  1]])

    # TF_2 = np.array([[11, 12, 13, 14],
    #                  [21, 22, 23, 24],
    #                  [31, 32, 33, 34],
    #                  [0,  0,  0,  1]])
    received_list = [float(item) for item in list_as_tf1.split(',')]
    TF_1 = np.array(received_list).reshape(4, 4)
    received_list = [float(item) for item in list_as_tf2.split(',')]
    TF_2 = np.array(received_list).reshape(4, 4)

    # P1 = np.array([[1,2,3],
    #                [2,3,4]])
    # P2 = np.array([[1,2,3],
    #                [2,3,4]])
    received_list = [float(item) for item in list_as_p1.split(',')]
    P1 = np.array(received_list).reshape(int(len(received_list) / 3), 3)
    received_list = [float(item) for item in list_as_p2.split(',')]
    P2 = np.array(received_list).reshape(int(len(received_list) / 3), 3)

    # 为变量提供一个初始猜测值
    # initial_guess = [1, 2, 3, 4, 5, 6, 7, 8 ,9 ,10 ,11 ,12]
    initial_guess = [float(item) for item in list_as_init.split(',')]

    # 定义总的残差函数，包括所有方程
    def total_residuals(vars):
        x11 = vars[0]
        x12 = vars[1]
        x13 = vars[2]
        x14 = vars[3]
        x21 = vars[4]
        x22 = vars[5]
        x23 = vars[6]
        x24 = vars[7]
        x31 = vars[8]
        x32 = vars[9]
        x33 = vars[10]
        x34 = vars[11]

        # 待优化的矩阵
        # x11 x12 x13 x14
        # x21 x22 x23 x24
        # x31 x32 x33 x34
        #   0   0   0   1
        # 3*3旋转矩阵行列向量的正交性和单位长度
        res = []
        res.append(x11 * x21 + x12 * x22 + x13 * x23)
        res.append(x11 * x31 + x12 * x32 + x13 * x33)
        res.append(x21 * x31 + x22 * x32 + x23 * x33)
        res.append(x11 * x12 + x21 * x22 + x31 * x32)
        res.append(x11 * x13 + x21 * x23 + x31 * x33)
        res.append(x12 * x13 + x22 * x23 + x32 * x33)

        res.append(x11 * x11 + x12 * x12 + x13 * x13 - 1)
        res.append(x21 * x21 + x22 * x22 + x23 * x23 - 1)
        res.append(x31 * x31 + x32 * x32 + x33 * x33 - 1)
        res.append(x11 * x11 + x21 * x21 + x31 * x31 - 1)
        res.append(x12 * x12 + x22 * x22 + x32 * x32 - 1)
        res.append(x13 * x13 + x23 * x23 + x33 * x33 - 1)
        # 3*3旋转矩阵行列式值为1
        res.append(x11 * (x22 * x33 - x23 * x32) - x12 * (x21 * x33 - x23 * x31) + x13 * (x21 * x32 - x22 * x31) - 1)

        # 一个特征点对应一组解
        points_num = P1.shape[0]
        for i in range(points_num):
            x1 = P1[i, 0]
            y1 = P1[i, 1]
            z1 = P1[i, 2]
            x2 = P2[i, 0]
            y2 = P2[i, 1]
            z2 = P2[i, 2]
            res.append(TF_1[0, 0] * (x1 * x11 + y1 * x12 + z1 * x13 + x14) +
                       TF_1[0, 1] * (x1 * x21 + y1 * x22 + z1 * x23 + x24) +
                       TF_1[0, 2] * (x1 * x31 + y1 * x32 + z1 * x33 + x34) + TF_1[0, 3]
                       -
                       TF_2[0, 0] * (x2 * x11 + y2 * x12 + z2 * x13 + x14) -
                       TF_2[0, 1] * (x2 * x21 + y2 * x22 + z2 * x23 + x24) -
                       TF_2[0, 2] * (x2 * x31 + y2 * x32 + z2 * x33 + x34) - TF_2[0, 3])
            res.append(TF_1[1, 0] * (x1 * x11 + y1 * x12 + z1 * x13 + x14) +
                       TF_1[1, 1] * (x1 * x21 + y1 * x22 + z1 * x23 + x24) +
                       TF_1[1, 2] * (x1 * x31 + y1 * x32 + z1 * x33 + x34) + TF_1[1, 3]
                       -
                       TF_2[1, 0] * (x2 * x11 + y2 * x12 + z2 * x13 + x14) -
                       TF_2[1, 1] * (x2 * x21 + y2 * x22 + z2 * x23 + x24) -
                       TF_2[1, 2] * (x2 * x31 + y2 * x32 + z2 * x33 + x34) - TF_2[1, 3])
            res.append(TF_1[2, 0] * (x1 * x11 + y1 * x12 + z1 * x13 + x14) +
                       TF_1[2, 1] * (x1 * x21 + y1 * x22 + z1 * x23 + x24) +
                       TF_1[2, 2] * (x1 * x31 + y1 * x32 + z1 * x33 + x34) + TF_1[2, 3]
                       -
                       TF_2[2, 0] * (x2 * x11 + y2 * x12 + z2 * x13 + x14) -
                       TF_2[2, 1] * (x2 * x21 + y2 * x22 + z2 * x23 + x24) -
                       TF_2[2, 2] * (x2 * x31 + y2 * x32 + z2 * x33 + x34) - TF_2[2, 3])

        return res


    # 使用least_squares函数求解
    result = least_squares(total_residuals, initial_guess)

    # 输出结果
    str_list = [str(item) for item in result.x]
    print(",".join(str_list))


