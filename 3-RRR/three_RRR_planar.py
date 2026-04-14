
import math

RAD_DEG = 180 / math.pi


class arm(object):
    """一个3-RRR并联机器人类，关节皆为转动副，专注运动模型

    Attributes:
        L: 机器人杆件长度组成的列表，前三个为支链杆件长度，第四个为支链原点所在圆的半径；
        tip_x_y: 机器人末端坐标系原点在全局坐标系下的坐标[x, y]；
        theta: 支链3个关节角度，弧度制，关节角度相对于初始姿态逆时针为正，顺时针为负（正方向为使末端位置向上运动的方向）；
        phi: 机器人末端姿态角
    Methods:
        __init__: 对象初始化函数；
        inverse_kinematics：根据末端件坐标系位姿求解关节角度的函数（逆解）；
        trans_coordinate: 全局坐标系到局部坐标系的坐标和姿态变换。
    """

    L = [0, 0, 0, 0] # 前三个为支链杆件长度，第四个为支链原点所在圆的半径
    tip_x_y = [0, 0] # 定义tip_x_y
    theta = [0, 0, 0] # 支链3个关节角度，弧度制
    phi = 0 # 末端姿态角，角度制

    def __init__(self, l=[20, 88.5, 104, 100]):
        """初始化函数，初始化机器人对象的属性参数 L。

        Args:
            l: 机器人杆件长度组成的列表，前三个为支链杆件长度，第四个为支链原点所在圆的半径；
        Returns:
            无。
        Raises:
            无。
        """
        self.L = l[:]

    def inverse_kinematics(self, tip_x_y=[0, 0], phi=0, ud=1):
        """根据末端件坐标系原点位置和姿态角求解关节角的函数，并以弧度制保存在theta列表中(运动学逆解)。

        Args:
            ud: (up/down)用来选择逆解中的第几种解：
                ud = 0 时(默认值)，机器人第三个关节在末端点上方,对应课程中的alpha构型；
                ud = 1 时，机器人第三个关节在末端点下方，对应课程中的beta构型.
        Returns:
            True: 求解过程顺利通过时，返回True；
            False: 当求解过程出现异常或者末端点超出工作空间时，返回False。
        Raises:
            无。
        """

        x = tip_x_y[0] - 0 # 消除模型坐标与实际坐标的误差
        y = tip_x_y[1] - 0 # 消除模型坐标与实际坐标的误差
        phi_nei = phi/180 * math.pi # 将姿态角换成成弧度制
        l1 = self.L[0]
        l2 = self.L[1]
        l3 = self.L[2]
        A = x - l3 * math.cos(phi_nei)
        B = y - l3 * math.sin(phi_nei)
        C = ((A ** 2 + B ** 2 + l1 ** 2 - l2 ** 2)) / (2 * l1)
        delta = A ** 2 + B ** 2 - C ** 2  # 设一个中间变量，以减少程序运算量
        if delta >= 0:
            if A == 0 and B == 0 and C == 0:
                print("支链第3关节和第1关节轴线重合")
                return False
            else:
                if A + C == 0:
                    t = - (A - C) / (2 * B)
                else:
                    if ud == 0:
                        t = (B + math.sqrt(delta)) / (A + C)
                    else:
                        t = (B - math.sqrt(delta)) / (A + C) # 经实验选取该值会导致后续角度无解
                self.theta[0] = 2 * math.atan(t)  # 求出驱动关节角度seta1
            D = (B - l1 * math.sin(self.theta[0])) / l2
            E = (A - l1 * math.cos(self.theta[0])) / l2
            self.theta[1] = math.atan2(D, E) - self.theta[0] # 求出从动关节角度seta2
            self.theta[2] = phi_nei - self.theta[0] - self.theta[1] # 求出从动关节角度seta3
            print(self.theta * 180)
            if abs(self.theta[2] / math.pi * 180 -0) < 5 or abs(self.theta[1] / math.pi * 180 -0) < 5: # 检查奇异位置
                print("支链接近奇异位置")
                return False
        else:
            print('末端位姿超出工作空间')
            return False
        return True

    def trans_coordinate(self, tip_x_y=[0, 0], phi=0, num=1):
        """根据机器人构型进行全局坐标系到局部坐标系的坐标和姿态变换。

        Args:
            tip_x_y: 末端在全局坐标系中的坐标
                phi: 末端在全局坐标系中的姿态角
                num: 支链编号
        Returns:
            True: 求解过程顺利通过时，返回True；
            False: 当求解过程出现异常或者末端点超出工作空间时，返回False。
        Raises:
            无。
        """

        x = tip_x_y[0]
        y = tip_x_y[1]
        R = self.L[3] # 三条支链原点所在圆半径
        if num == 1:
            x1 = y + R/2
            y1 = - x + math.sqrt(3)/2 * R
            phi1 = phi + 180 # 根据初始姿态角在全局坐标系和局部坐标系中的差值决定
            return [x1, y1, phi1]
        if num == 2:
            x2 = - math.sqrt(3)/2 * x - y/2 + R/2
            y2 = x/2 - math.sqrt(3)/2 * y + math.sqrt(3)/2 * R
            phi2 = phi + 180 # 根据初始姿态角在全局坐标系和局部坐标系中的差值决定
            return [x2, y2, phi2]
        if num == 3:
            x3 = math.sqrt(3)/2 * x - y/2 + R/2
            y3 = x/2 + math.sqrt(3)/2 * y + math.sqrt(3)/2 * R
            phi3 = phi + 180
            return [x3, y3, phi3]
        if num < 1 or num > 3:
            print("支链编号 num 参数错误，请输入 1~3")
            return False