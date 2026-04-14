import math as cm
import Dr_3_RRR as robot

max_list_temp = [90, 90, 90]
min_list_temp = [-90, -90, -90]
l_temp = [150, 150, 115.25, 184.75]
default_phi = -30

ro = robot.robot(MAX_list_temp=max_list_temp, MIN_list_temp=min_list_temp, L_temp=l_temp)


def draw_rectangle(pl=[-25, 35], l=50, h=50, n=80):
    """生成矩形轨迹点。"""

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


def draw_concentric_circles(center=[0, 0], radii=None, n=180, overlap_angle_deg=40):
    """生成同心圆轨迹点，每个圆单独保存。"""

    if radii is None:
        radii = [15, 30]

    circle_list = []
    total_angle = 2 * cm.pi + overlap_angle_deg / 180 * cm.pi
    angle_delta = total_angle / n

    for radius in radii:
        one_circle = []
        for i in range(0, n + 1):
            x = center[0] + radius * cm.cos(angle_delta * i)
            y = center[1] + radius * cm.sin(angle_delta * i)
            one_circle.append([x, y])
        one_circle.append(one_circle[0][:])
        one_circle.append(one_circle[0][:])
        circle_list.append(one_circle)

    return circle_list


def draw_triangle(p1=[-20, -5], p2=[20, -5], p3=[0, 29.64], n=100):
    """生成正三角形轨迹点。"""

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


def run_curve(pl_list, phi=default_phi, start_speed=10, run_time=10, loop=True):
    """执行单条闭合轨迹。"""

    ro.set_poses_curve_pre(tip_x_ys=pl_list, phis=[phi])
    ro.set_poses_curve_start_point(start_speed)

    if loop:
        while True:
            ro.set_poses_curve_do(run_time)
    else:
        ro.set_poses_curve_do(run_time)


def run_concentric_circles(circle_list, phi=default_phi, move_speed=10, run_time=8, loop=True):
    """逐个执行同心圆，保证每个圆独立闭合。"""

    while True:
        for one_circle in circle_list:
            ro.set_pose(tip_x_y=one_circle[0], phi=phi, speed=move_speed, acceleration=10)
            ro.pose_done()
            ro.set_poses_curve_pre(tip_x_ys=one_circle, phis=[phi])
            ro.set_poses_curve_do(run_time)
            ro.pose_done()
        if not loop:
            break


if __name__ == "__main__":
    # 选择要画的图形：'rectangle'、'circles'、'triangle'
    shape = "rectangle"

    if shape == "rectangle":
        points = draw_rectangle(pl=[-25, 35], l=50, h=50, n=80)
        run_curve(points, phi=default_phi, start_speed=10, run_time=10, loop=True)
    elif shape == "circles":
        circles = draw_concentric_circles(center=[0, 0], radii=[15, 30], n=180)
        run_concentric_circles(circles, phi=default_phi, move_speed=10, run_time=8, loop=True)
    elif shape == "triangle":
        side = 40
        height = side * cm.sqrt(3) / 2
        points = draw_triangle(
            p1=[-side / 2, -height / 3],
            p2=[side / 2, -height / 3],
            p3=[0, 2 * height / 3],
            n=100,
        )
        run_curve(points, phi=default_phi, start_speed=10, run_time=10, loop=True)
    else:
        raise ValueError("shape 只能是 'rectangle'、'circles' 或 'triangle'")
