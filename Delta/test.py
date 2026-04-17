import math as cm
import DrDelta as robot

max_list_temp = [90, 90, 90]
min_list_temp = [-42, -42, -42]
l_temp = [100, 250, 35, 23.4]

safe_z = -237
draw_plane_z = safe_z

ro = robot.robot(MAX_list_temp=max_list_temp, MIN_list_temp=min_list_temp, L_temp=l_temp)


def draw_rectangle(pl=[-30, 30, draw_plane_z], l=60, h=40, n=80):
    """生成矩形轨迹点。"""

    pl_list = [pl[:]]
    l_delta = l / n
    h_delta = h / n

    for i in range(1, n + 1):
        pl_list.append([pl[0] + i * l_delta, pl[1], pl[2]])
    for i in range(1, n + 1):
        pl_list.append([pl[0] + l, pl[1] - i * h_delta, pl[2]])
    for i in range(1, n + 1):
        pl_list.append([pl[0] + l - i * l_delta, pl[1] - h, pl[2]])
    for i in range(1, n + 1):
        pl_list.append([pl[0], pl[1] - h + i * h_delta, pl[2]])

    return pl_list


def draw_concentric_circles(center=[0, 0, draw_plane_z], radii=None, n=180, overlap_angle_deg=40):
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
            one_circle.append([x, y, center[2]])
        one_circle.append(one_circle[0][:])
        one_circle.append(one_circle[0][:])
        circle_list.append(one_circle)

    return circle_list


def draw_triangle(p1=[-20, -11.55, draw_plane_z], p2=[20, -11.55, draw_plane_z], p3=[0, 23.09, draw_plane_z], n=100):
    """生成正三角形轨迹点。"""

    def segment_points(start, end, point_num):
        points = []
        for i in range(0, point_num + 1):
            ratio = i / point_num
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            z = start[2] + (end[2] - start[2]) * ratio
            points.append([x, y, z])
        return points

    pl_list = []
    pl_list.extend(segment_points(p1, p2, n))
    pl_list.extend(segment_points(p2, p3, n))
    pl_list.extend(segment_points(p3, p1, n))
    return pl_list


def run_curve(pl_list, safe_z=-200, start_speed=10, run_time=10, loop=True):
    """执行单条闭合轨迹。"""

    ro.set_positions_curve_pre(tip_x_y_zs=pl_list)
    ro.set_position(tip_x_y_z=[0, 0, safe_z], speed=start_speed, acceleration=10)
    ro.position_done()
    ro.set_positions_curve_start_point(start_speed)

    if loop:
        while True:
            ro.set_positions_curve_do(run_time)
    else:
        ro.set_positions_curve_do(run_time)


def run_concentric_circles(circle_list, safe_z=-200, move_speed=10, run_time=8, loop=True):
    """逐个执行同心圆，保证每个圆独立闭合。"""

    while True:
        for one_circle in circle_list:
            ro.set_position(tip_x_y_z=[0, 0, safe_z], speed=move_speed, acceleration=10)
            ro.position_done()
            ro.set_position(tip_x_y_z=one_circle[0], speed=move_speed, acceleration=10)
            ro.position_done()
            ro.set_positions_curve_pre(tip_x_y_zs=one_circle)
            ro.set_positions_curve_do(run_time)
            ro.position_done()
        if not loop:
            break


if __name__ == "__main__":
    # 选择要画的图形：'rectangle'、'circles'、'triangle'
    shape = "rectangle"

    if shape == "rectangle":
        points = draw_rectangle(pl=[-30, 30, draw_plane_z], l=60, h=40, n=80)
        run_curve(points, safe_z=safe_z, start_speed=10, run_time=10, loop=True)
    elif shape == "circles":
        circles = draw_concentric_circles(center=[0, 0, draw_plane_z], radii=[15, 30], n=180)
        run_concentric_circles(circles, safe_z=safe_z, move_speed=10, run_time=8, loop=True)
    elif shape == "triangle":
        side = 40
        height = side * cm.sqrt(3) / 2
        points = draw_triangle(
            p1=[-side / 2, -height / 3, draw_plane_z],
            p2=[side / 2, -height / 3, draw_plane_z],
            p3=[0, 2 * height / 3, draw_plane_z],
            n=100,
        )
        # noinspection PyInterpreter
        run_curve(points, safe_z=safe_z, start_speed=10, run_time=10, loop=True)
    else:
        raise ValueError("shape 只能是 'rectangle'、'circles' 或 'triangle'")
