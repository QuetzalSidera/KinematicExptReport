
import math

RAD_DEG = 180 / math.pi


class arm(object):
    """一个五连杆并联机器人类，关节皆为转动副，专注运动模型

    Attributes:
        L: 机器人杆件长度组成的列表；
        tip_x_y: 机器人末端坐标系原点在全局坐标系下的坐标[x, y, z]；
        theta: 2个驱动关节角度，弧度制，关节角度相对于初始姿态逆时针为正，顺时针为负（正方向为使末端位置向上运动的方向）；
    Methods:
        __init__: 对象初始化函数；
        inverse_kinematics：根据末端件坐标系位姿求解关节角度的函数（逆解）；
        forward_kinematics_pose: 根据关节角度求解末端点坐标系位姿函数（正解）；
    """
    L = [0, 0, 0, 0, 0] # 五根杆件长度
    tip_x_y = [0, 0] # 定义末端位置坐标 tip_x_y
    theta = [0, 0] # 2 个驱动关节角度，弧度制
    x_err = -1 # 模型坐标与实际坐标在 x 轴方向上的静态误差
    y_err = 0  # 模型坐标与实际坐标在 y 轴方向上的静态误差

    def __init__(self, l=[100, 100, 100, 100, 200]):
        """初始化函数，初始化机器人对象的属性参数 L

        Args:
            l: 机器人杆件长度组成的列表。
        Returns:
            无。
        Raises:
            无。
        """
        self.L = l[:]

    def inverse_kinematics(self, tip_x_y=[0, 0], ud=1):
        """根据末端位置求解关节角的函数，并以弧度制保存在theta列表中(运动学逆解)。

        Args:
            tip_x_y：末端位置坐标
            ud: (up/down)用来选择逆解中的第几种解：
                ud = 0 时(默认值)，机器人第三个关节在末端点上方,对应课程中的alpha构型；
                ud = 1 时，机器人第三个关节在末端点下方，对应课程中的beta构型.
        Returns:
            True: 求解过程顺利通过时，返回True；
            False: 当求解过程出现异常或者末端点超出工作空间时，返回False。
        Raises:
            无。
        """

        theta_bk = self.theta.copy()
        x = tip_x_y[0] + self.x_err # 消除模型坐标与实际坐标的误差
        y = tip_x_y[1] + self.y_err # 消除模型坐标与实际坐标的误差
        l1 = self.L[0]
        l2 = self.L[1]
        l3 = self.L[2]
        l4 = self.L[3]
        s = self.L[4]
        x_a = - s
        A1 = 2 * l1 * (x - x_a)
        B1 = 2 * y * l1
        C1 = x ** 2 + y ** 2 + x_a ** 2 + l1 ** 2 - l2 ** 2 - 2 * x * x_a
        delta1 = A1 ** 2 + B1 ** 2 - C1 ** 2  # 设一个中间变量，以减少程序运算量
        if delta1 >= 0:
            if A1 == 0 and B1 == 0 and C1 == 0:
                self.theta[0] = theta_bk[0] # 针对机器人杆l1与杆l2长度相等，且末端点支链原点处的情况，关节1保持上一次的值
            else:
                if A1 + C1 == 0:
                    t1 = - (A1 - C1) / (2 * B1)
                else:
                    if ud == 0:
                        t1 = (B1 + math.sqrt(delta1)) / (A1 + C1)
                    else:
                        t1 = (B1 - math.sqrt(delta1)) / (A1 + C1) # 经实验选取该值会导致后续角度无解
                self.theta[0] = 2 * math.atan(t1)  # 求出seta1
                if self.theta[0] < 0:
                    self.theta[0] += 2 * math.pi # 1 号关节的转动范围为 0~360 度
        else:
            print('支链1末端位姿超出工作空间')
            self.theta = theta_bk.copy()
            return False
        x_b = s
        A2 = 2 * l3 * (x - x_b)
        B2 = 2 * y * l3
        C2 = x ** 2 + y ** 2 + x_b ** 2 + l3 ** 2 - l4 ** 2 - 2 * x * x_b
        delta2 = A2 ** 2 + B2 ** 2 - C2 ** 2  # 设一个中间变量，以减少程序运算量
        if delta2 >= 0:
            if A2 == 0 and B2 == 0 and C2 == 0:
                self.theta[1] = theta_bk[1]  # 针对机器人杆l3与杆l4长度相等，且末端点支链原点处的情况，关节2保持上一次的值
            else:
                if A2 + C2 == 0:
                    t2 = - (A2 - C2) / (2 * B2)
                else:
                    if ud == 0:
                        t2 = (B2 - math.sqrt(delta2)) / (A2 + C2)
                    else:
                        t2 = (B2 + math.sqrt(delta2)) / (A2 + C2)  # 经实验选取该值会导致后续角度无解
                self.theta[1] = 2 * math.atan(t2)  # 求出seta2
        else:
            print('支链2末端位姿超出工作空间')
            self.theta = theta_bk.copy()
            return False
        x_c = x_a + l1 * math.cos(self.theta[0])
        y_c = l1 * math.sin(self.theta[0])
        x_d = x_b + l3 * math.cos(self.theta[1])
        y_d = l3 * math.sin(self.theta[1])
        l_C_D = math.sqrt((x_c - x_d) ** 2 + (y_c - y_d) ** 2)
        if ((l2 + l4) - l_C_D) < 1: # 通过C、D两点间的距离判断是否到达奇异位置
            print("接近奇异位置")
            self.theta = theta_bk.copy()
            return False
        # if l_C_D < 40:
        #     print("C、D两点即将碰撞")
        #     self.theta = theta_bk.copy()
        #     return False
        return True

    def forward_kinematics_position(self, angle_list=[0, 90], ud=1):
        """根据驱动关节角求解末端点位置(运动学正解)

        Args:
            angle_list: 驱动关节角度组成的列表[joint1,joint2]（角度制）
        Returns:
            无
        Raises:
            无
        """

        angle = [math.pi / 180 * i for i in angle_list]
        seta1 = angle[0]
        seta2 = angle[1]
        l1 = self.L[0]
        l2 = self.L[1]
        l3 = self.L[2]
        l4 = self.L[3]
        s = self.L[4]
        x_a = -s
        x_c = x_a + l1 * math.cos(seta1)
        y_c = l1 * math.sin(seta1)
        x_b = s
        x_d = x_b + l3 * math.cos(seta2)
        y_d = l3 * math.sin(seta2)
        R1 = l2 ** 2 - x_c ** 2 - y_c ** 2
        R2 = l4 ** 2 - x_d ** 2 - y_d ** 2
        if x_c != x_d:
            R3 = (R1 - R2) / (2 * (x_d - x_c))
            R4 = - (y_d - y_c) / (x_d - x_c)
            A = R4 ** 2 + 1
            B = -2 * (x_c * R4 - R3 * R4 + y_c)
            C = R3 ** 2 - 2 * x_c * R3 - R1
            delta1 = B ** 2 - 4 * A * C
            if delta1 >= 0:
                if ud == 0:
                    y = (-B - math.sqrt(delta1)) / (2 * A)
                else:
                    y = (-B + math.sqrt(delta1)) / (2 * A)
                x = R3 + R4 * y
            else:
                print("该构型无解")
                return False
        else:
            if y_c != y_d:
                y = (R1 - R2) / (2 * (y_d - y_c))
                D = -2 * x_c
                E = y ** 2 - y_c * y - R1
                delta2 = D ** 2 - 4 * E
                if delta2 >= 0:
                    if ud == 0:
                        x = (-D + math.sqrt(delta2)) / 2
                    else:
                        x = (-D - math.sqrt(delta2)) / 2
                else:
                    print("该构型无解")
                    return False
            else:
                print("杆件干涉")
                return False
        # self.tip_x_y = [x - self.x_err, y - self.y_err].copy()
        self.tip_x_y = [x, y].copy()
        return True





