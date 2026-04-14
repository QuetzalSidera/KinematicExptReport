import math as cm
import Dr_5_bar as robot

max_list_temp = [360, 180]  # 关节模型角度最大值
min_list_temp = [0, -180]  # 关节模型角度最小值
l_temp = [150, 225, 150, 225, 75]  # 机器人尺寸参数：[l1, l2, l3, l4, l5]

# 机械臂对象初始化
ro = robot.robot(MAX_list_temp=max_list_temp, MIN_list_temp=min_list_temp, L_temp=l_temp)


def draw_rectangle(pl=[-50, 250], l=120, h=80, n=80):
    """生成矩形轨迹点。

    pl: 左上角坐标 [x, y]
    l: 矩形宽度
    h: 矩形高度
    n: 每条边的分点数量，越大越平滑，但也越慢
    """

    pl_list = [pl[:]]
    l_delta = l / n
    h_delta = h / n

    for i in range(1, n + 1):
        pl_list.append([pl[0] + i * l_delta, pl[1]])
    for i in range(1, n + 1):
        pl_list.append([pl[0] + l, pl[1] - i * h_delta])
    for i in range(1, n + 1):
        pl_list.append([pl[0] + l - i * l_delta, pl[1] - h])
    for i in range(1, n + 1):
        pl_list.append([pl[0], pl[1] - h + i * h_delta])

    return pl_list


def draw_concentric_circles(center=[0, 220], radii=None, n=180, overlap_angle_deg=40):
    """生成同心圆轨迹点，每个圆单独保存。

    center: 圆心坐标 [x, y]
    radii: 半径列表，比如 [20, 40, 60]
    n: 每个圆的分点数量
    overlap_angle_deg: 每个圆额外多转的角度，避免收尾出现缺口
    """

    if radii is None:
        radii = [20, 40, 60]

    circle_list = []
    total_angle = 2 * cm.pi + overlap_angle_deg / 180 * cm.pi
    angle_delta = total_angle / n

    for radius in radii:
        one_circle = []
        for i in range(0, n + 1):
            x = center[0] + radius * cm.cos(angle_delta * i)
            y = center[1] + radius * cm.sin(angle_delta * i)
            one_circle.append([x, y])
        # 收尾时再压回起点几次，像画图时故意多描一点，减少缺口
        one_circle.append(one_circle[0][:])
        one_circle.append(one_circle[0][:])
        circle_list.append(one_circle)

    return circle_list


def draw_triangle(p1=[-60, 240], p2=[60, 240], p3=[0, 320], n=100):
    """生成三角形轨迹点。

    p1, p2, p3: 三角形三个顶点坐标
    n: 每条边的分点数量
    """

    def segment_points(start, end, point_num):
        points = []
        for i in range(0, point_num + 1):
            ratio = i / point_num
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            points.append([x, y])
        return points

    pl_list = []
    pl_list.extend(segment_points(p1, p2, n))
    pl_list.extend(segment_points(p2, p3, n))
    pl_list.extend(segment_points(p3, p1, n))
    return pl_list


def run_curve(pl_list, start_speed=10, run_time=20, loop=True):
    """执行轨迹。"""

    ro.set_positions_curve_pre(tip_x_ys=pl_list)
    ro.set_positions_curve_start_point(start_speed)

    if loop:
        while True:
            ro.set_positions_curve_do(run_time)
    else:
        ro.set_positions_curve_do(run_time)


def run_concentric_circles(circle_list, move_speed=10, run_time=12, loop=True):
    """逐个执行同心圆，保证每个圆都是独立闭合的。"""

    while True:
        for one_circle in circle_list:
            ro.set_position(tip_x_y=one_circle[0], speed=move_speed, acceleration=20)
            ro.position_done()
            ro.set_positions_curve_pre(tip_x_ys=one_circle)
            ro.set_positions_curve_do(run_time)
            ro.position_done()
        if not loop:
            break


if __name__ == "__main__":
    # 选择要画的图形：'rectangle'、'circles'、'triangle'
    shape = "rectangle"

    if shape == "rectangle":
        points = draw_rectangle(pl=[-80, 260], l=50, h=50, n=80)
        run_curve(points, start_speed=10, run_time=20, loop=True)
    elif shape == "circles":
        circles = draw_concentric_circles(center=[0, 240], radii=[20, 40, 60], n=180)
        run_concentric_circles(circles, move_speed=10, run_time=12, loop=True)
    elif shape == "triangle":
        points = draw_triangle(
            p1=[-10, 220 - 20 * cm.sqrt(3) / 6],
            p2=[10, 220 - 20 * cm.sqrt(3) / 6],
            p3=[0, 220 + 20 * cm.sqrt(3) / 3],
            n=100,
        )
        run_curve(points, start_speed=10, run_time=20, loop=True)
    else:
        raise ValueError("shape 只能是 'rectangle'、'circles' 或 'triangle'")
