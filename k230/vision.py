"""
视觉检测模块 - 2023电赛E题
移植自开源项目 kaka12331/2023-dian-sai-visual-gimbal
适配方案A: K230直连F32C控电机

功能:
- 同心矩形检测 (屏幕0.5m正方形 / A4胶带)
- 绿色激光点检测 (LAB色彩空间)
- 阈值调节预览 (触屏调参)
- 路径插值 (4角点间生成轨迹)
"""
import math
import config

# ============================================================
# 全局状态 (检测结果显示给主循环)
# ============================================================
# ROI
_roi_w = config.LCD_W // 2
_roi_h = config.LCD_H // 2
_roi_x = (config.LCD_W - _roi_w) // 2
_roi_y = (config.LCD_H - _roi_h) // 2
DEFAULT_ROI_PTS = [(_roi_x, _roi_y),
                   (_roi_x + _roi_w, _roi_y),
                   (_roi_x + _roi_w, _roi_y + _roi_h),
                   (_roi_x, _roi_y + _roi_h)]
ROI_CENTER = (config.LCD_W // 2, config.LCD_H // 2)

# 运行时可变状态
cur_roi = (_roi_x, _roi_y, _roi_w, _roi_h)
cur_roi_pts = DEFAULT_ROI_PTS[:]

# 检测结果
rect_coords  = []       # 同心矩形4角点 (平均后的中线点)
red_coords   = []       # 红色激光点坐标 (cx, cy)
green_coords = []       # 绿色激光点坐标
spot_area    = 0        # 采纳的光斑块面积(px) —— 显示出来才能看出是不是把假块当真光斑了

# 🔧 光斑块面积的合理上限: 光斑约16px, 假块(阴影/反光)几百上千px.
#    超过就判为假块不采纳 —— 宁可这一帧没读数, 也不让伺服去追假块
SPOT_MAX_AREA = 200

# 路径插值
path_points = []
path_idx = 0

# 调试信息 (排查矩形检测不稳定用)
# raw=find_rects原始数量, area=面积筛选后, angfail=角度校验失败数, hold=保持帧数(0=本帧新检测)
detect_debug = {"raw": 0, "area": 0, "angfail": 0, "hold": 0}

# 光斑检出率(最近N帧): 用来分清"激光跟不上蓝点"是**检测丢了**还是**伺服跟不上**
# 伺服要连续SERVO_MED_N帧都检出才给反馈, 所以检出率一低伺服就等于开环
SPOT_RATE_N = 60          # 18fps下约3.3秒
spot_hist = []

# 检测保持: find_rects帧间抖动/轻微移动导致偶发丢帧时, 沿用上一次的好框
last_rect = []        # 上一次有效矩形4角点
hold_count = 0        # 已连续保持的帧数


# ============================================================
# 工具函数 (移植自开源项目)
# ============================================================
def sort_corners_clockwise(points):
    """将4个点按逆时针排序，保证一致的方向"""
    if len(points) != 4:
        return points
    cx = sum(p[0] for p in points) / 4
    cy = sum(p[1] for p in points) / 4
    return sorted(points, key=lambda p: math.atan2(p[1] - cy, p[0] - cx), reverse=False)


def draw_rect(img, corners, color=(0, 255, 0), draw_dots=True, thick=2):
    """绘制矩形边 + 可选角点圆"""
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        img.draw_line(a[0], a[1], b[0], b[1], color, thick)
    if draw_dots:
        for x, y in corners:
            img.draw_circle(x, y, 5, (0, 0, 255), thick, fill=False)


def is_rectangular(corners, tol=45):
    """检查四个角是否都接近90°（tol为容许偏差度），过滤畸形四边形"""
    for i in range(4):
        a = corners[i]
        b = corners[(i + 1) % 4]
        c = corners[(i + 2) % 4]
        v1 = (a[0] - b[0], a[1] - b[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        m1 = math.sqrt(v1[0]**2 + v1[1]**2)
        m2 = math.sqrt(v2[0]**2 + v2[1]**2)
        if m1 < 2 or m2 < 2:
            return False
        cos_a = max(-1, min(1, dot / (m1 * m2)))
        if abs(math.degrees(math.acos(cos_a)) - 90) > tol:
            return False
    return True


def _lab_t_idx(param_idx, color_mode):
    """将 LAB 参数索引 (0-5) 映射到 T[] 实际索引
       color_mode: 0=红色(T[6-11]), 1=绿色(T[12-17])"""
    return (6 if color_mode == 0 else 12) + param_idx


def generate_path(pts, n):
    """在4个角点之间线性插值生成路径点（含角点），共 4*(n+1) 个点"""
    path = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        path.append(p1)
        for j in range(1, n + 1):
            alpha = j / (n + 1)
            x = int(p1[0] + (p2[0] - p1[0]) * alpha)
            y = int(p1[1] + (p2[1] - p1[1]) * alpha)
            path.append((x, y))
    return path


# ============================================================
# 视觉检测函数 (移植自开源项目)
# ============================================================
def update_roi():
    """更新 ROI (根据 roi_scale 缩放)"""
    global cur_roi, cur_roi_pts
    s = config.VISION_T[5]
    cx, cy = ROI_CENTER
    cur_roi_pts = [(int(cx + (x - cx) * s), int(cy + (y - cy) * s)) for (x, y) in DEFAULT_ROI_PTS]
    xs = [p[0] for p in cur_roi_pts]
    ys = [p[1] for p in cur_roi_pts]
    cur_roi = (min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))


def run_detect(img):
    """同心矩形检测 + 绿色激光检测 (题3/4用)
    在 img 上绘制检测结果（修改原图）
    """
    global rect_coords, red_coords, path_points, path_idx, last_rect, hold_count, spot_area
    T = config.VISION_T

    update_roi()

    # 检测绿色激光 (LAB色彩空间, T[12-17])
    # 🔧 stride=1: 光斑在画面里只有1~2px(5mm光斑@3.3px/cm), 隔像素扫(x_stride=2)会扫出
    #    残缺形状 -> 质心帧间跳1~4px. 这点精度对2cm容差是致命的, 所以不省这点算力
    blobs = img.find_blobs([(T[12], T[13], T[14], T[15], T[16], T[17])], False, roi=cur_roi,
                           x_stride=1, y_stride=1, pixels_threshold=1,
                           merge=True, margin=True)
    # 🔧 面积闸门: find_blobs 取的是"面积最大的块", 而阈值一旦放宽(比如把 L_min 放到0),
    #    暗绿色的阴影/反光会凑出几百上千px的假块并把真光斑(约16px)压下去 ->
    #    伺服去追那个假块 = 一阵暴发式的偏出去(实测 maxerr 从14px恶化到41px).
    #    光斑不可能有 SPOT_MAX_AREA 那么大, 超过的直接判为假块, 不采纳.
    if blobs:
        blobs = [b for b in blobs if b.area() <= SPOT_MAX_AREA]
    if blobs:
        best = max(blobs, key=lambda b: (b.area(), -b.y()))
        red_coords = (best.cx(), best.cy())
        spot_area = best.area()
        img.draw_cross(best.cx(), best.cy(), (0, 255, 0), 20, 3)
        _push_spot_hit(1)
    else:
        red_coords = []
        spot_area = 0
        _push_spot_hit(0)

    # 灰度拷贝做二值化 + 膨胀 (避免绘图干扰)
    g = img.to_grayscale(copy=True)
    g.binary([(T[3], T[4])]).dilate(1)

    rects = g.find_rects(threshold=T[0], roi=cur_roi)
    detect_debug["raw"] = len(rects)
    # 面积筛选 + 按面积降序
    rects = [r for r in rects if T[1] <= r.rect()[2] * r.rect()[3] <= T[2]]
    rects.sort(key=lambda r: r.rect()[2] * r.rect()[3], reverse=True)
    detect_debug["area"] = len(rects)

    # 角度校验: 遍历所有候选, 取面积最大且角度合格的
    # (旧代码只查最大的一个; 最大框若被背景(书本/桌面边缘)带偏成畸形, 后面合格的框会被一起误丢)
    valid = []
    for r in rects:
        pts = sort_corners_clockwise(r.corners())
        if is_rectangular(pts):
            valid.append(pts)
    detect_debug["angfail"] = len(rects) - len(valid)

    if valid:
        # ── 本帧检测成功 ──
        outer = valid[0]
        if len(valid) >= 2:
            # 双矩形: 取内外框中线
            inner = valid[1]
            dists = [(inner[j][0] - outer[0][0])**2 + (inner[j][1] - outer[0][1])**2 for j in range(4)]
            start = dists.index(min(dists))
            inner = inner[start:] + inner[:start]
            draw_rect(img, outer)
            draw_rect(img, inner)
            corners = []
            for i in range(4):
                corners.append((
                    (outer[i][0] + inner[i][0]) // 2,
                    (outer[i][1] + inner[i][1]) // 2
                ))
        else:
            # 单矩形: 角度合格直接用外框
            corners = outer
            draw_rect(img, corners)
        rect_coords = corners
        last_rect = corners[:]
        hold_count = 0
        detect_debug["hold"] = 0
        path_points = generate_path(corners, config.PATH_N)
        if path_idx >= len(path_points):
            path_idx = 0
        for x, y in corners:
            img.draw_circle(x, y, 3, (255, 255, 0), 2, fill=True)
    elif last_rect and hold_count < config.RECT_HOLD_FRAMES:
        # ── 本帧丢失: 保持上一帧好框 (find_rects帧间抖动/轻微移动时画面和轨迹不中断) ──
        hold_count += 1
        detect_debug["hold"] = hold_count
        rect_coords = last_rect
        draw_rect(img, last_rect, (0, 255, 255), False, 2)  # 青色框=保持中的旧框
    else:
        # ── 彻底丢失 (保持超时) ──
        rect_coords = []
        detect_debug["hold"] = hold_count

    # ROI 框
    draw_rect(img, cur_roi_pts, (255, 255, 255), False, 2)
    del g


def run_func2(img):
    """功能2：同时检测红色激光和绿色激光 (追踪系统用, 本项目不做绿色)"""
    global red_coords, green_coords
    T = config.VISION_T

    # 红色激光
    red_blobs = img.find_blobs([(T[6], T[7], T[8], T[9], T[10], T[11])], False,
                               x_stride=2, y_stride=2, pixels_threshold=1,
                               merge=True, margin=True)
    if red_blobs:
        best = max(red_blobs, key=lambda b: (b.area(), -b.y()))
        red_coords = (best.cx(), best.cy())
        img.draw_cross(best.cx(), best.cy(), (0, 255, 0), 20, 3)
        img.draw_string_advanced(best.cx() + 15, best.cy() - 15, 20, "R", (0, 255, 0))
    else:
        red_coords = []

    # 绿色激光
    green_blobs = img.find_blobs([(T[12], T[13], T[14], T[15], T[16], T[17])], False,
                                 x_stride=2, y_stride=2, pixels_threshold=1,
                                 merge=True, margin=True)
    if green_blobs:
        best = max(green_blobs, key=lambda b: (b.area(), -b.y()))
        green_coords = (best.cx(), best.cy())
        img.draw_cross(best.cx(), best.cy(), (255, 0, 255), 20, 3)
        img.draw_string_advanced(best.cx() + 15, best.cy() - 15, 20, "G", (255, 0, 255))
    else:
        green_coords = []


def run_threshold_preview(img):
    """阈值调节预览：二值化 + 矩形 + 激光标记"""
    global cur_roi, cur_roi_pts
    T = config.VISION_T

    update_roi()

    # 检测绿色激光（在原图上, T[12-17]）
    blobs = img.find_blobs([(T[12], T[13], T[14], T[15], T[16], T[17])], False, roi=cur_roi,
                           x_stride=2, y_stride=2, pixels_threshold=1,
                           merge=True, margin=True)

    # 二值化预览
    g = img.to_grayscale(copy=True).binary([(T[3], T[4])]).dilate(1)
    rects = g.find_rects(threshold=T[0], roi=cur_roi)
    rects = [r for r in rects if T[1] <= r.rect()[2] * r.rect()[3] <= T[2]]
    if rects:
        for r in rects:
            pts = sort_corners_clockwise(r.corners())
            if is_rectangular(pts):
                draw_rect(g, pts)

    # 激光标记画到二值图上
    if blobs:
        best = max(blobs, key=lambda b: (b.area(), -b.y()))
        g.draw_cross(best.cx(), best.cy(), (0, 255, 255), 25, 3)

    draw_rect(g, cur_roi_pts, (255, 0, 0), False, 2)

    # 参数信息
    info1 = "L_min={}  L_max={}".format(T[3], T[4])
    g.draw_string_advanced(10, config.LCD_H - 55, 20, info1, color=(255, 255, 0))
    info2 = "roi_s={:.2f}".format(T[5])
    g.draw_string_advanced(10, config.LCD_H - 30, 20, info2, color=(255, 255, 0))
    return g


def run_lab_preview(img):
    """LAB 阈值二值化预览：逐像素 LAB 筛选，匹配=白，不匹配=黑"""
    global cur_roi, cur_roi_pts
    T = config.VISION_T

    update_roi()

    # 绿色 LAB 阈值
    lab_th = [(T[12], T[13], T[14], T[15], T[16], T[17])]

    # 🔧 顺带跑一次光斑检测: 标定时必须能一边按按钮一边看检出率有没有上100%,
    #    以及光斑块有多大(blob Npx) —— 只有1~2px说明阈值把光斑的光晕切掉了, 质心会乱跳
    blobs = img.find_blobs(lab_th, False, roi=cur_roi, x_stride=1, y_stride=1,
                           pixels_threshold=1, merge=True, margin=True)
    if blobs:
        best = max(blobs, key=lambda b: (b.area(), -b.y()))
        _push_spot_hit(1)
    else:
        best = None
        _push_spot_hit(0)

    # 创建彩色副本 → 逐像素 LAB 二值化
    import image
    g = image.Image(config.LCD_W, config.LCD_H, image.RGB565)
    g.draw_image(img, 0, 0)
    g.binary(lab_th)
    g = g.to_grayscale()

    # ROI 框
    draw_rect(g, cur_roi_pts, (100, 100, 100), False, 2)

    # 光斑标记 + 检出率 + 光斑块面积
    if best is not None:
        g.draw_cross(best.cx(), best.cy(), (0, 255, 255), 20, 3)
    # ⚠️ 这里显示的是**不过闸门**的最大块面积: 标定时要用它发现"假块"
    #    (真光斑约16px; 要是这里几百上千px, 说明阈值把阴影/反光也放进来了)
    g.draw_string_advanced(10, config.LCD_H - 80, 20,
        "spot {}%  biggest {}px".format(get_spot_rate(), best.area() if best else 0),
        color=(255, 255, 0))

    # 参数信息
    info = "GREEN: L={},{}, A={},{}, B={},{}".format(
        T[12], T[13], T[14], T[15], T[16], T[17])
    g.draw_string_advanced(10, config.LCD_H - 55, 20, info, color=(200, 200, 200))
    return g


def _push_spot_hit(hit):
    """记录本帧有没有检出光斑 (滚动窗口; 只 append/pop 不重新绑定, 所以不需要 global)"""
    spot_hist.append(hit)
    if len(spot_hist) > SPOT_RATE_N:
        spot_hist.pop(0)


def get_spot_rate():
    """最近SPOT_RATE_N帧的光斑检出率(0~100); 还没数据返回 -1"""
    if not spot_hist:
        return -1
    return sum(spot_hist) * 100 // len(spot_hist)


def get_red_coords():
    """获取红色激光点坐标"""
    return red_coords


def get_spot_area():
    """最近一帧采纳的光斑块面积(px); 0=本帧没检出"""
    return spot_area


def get_rect_coords():
    """获取同心矩形角点"""
    return rect_coords


def _line_intersection(a, b, c, d):
    """直线ab与直线cd的交点 (整数坐标), 平行返回None"""
    den = (a[0] - b[0]) * (c[1] - d[1]) - (a[1] - b[1]) * (c[0] - d[0])
    if den == 0:
        return None
    q1 = a[0] * b[1] - a[1] * b[0]
    q2 = c[0] * d[1] - c[1] * d[0]
    x = (q1 * (c[0] - d[0]) - (a[0] - b[0]) * q2) // den
    y = (q1 * (c[1] - d[1]) - (a[1] - b[1]) * q2) // den
    return int(x), int(y)


def _dist_to_seg(px, py, ax, ay, bx, by):
    """点到线段的距离"""
    dx = bx - ax
    dy = by - ay
    dd = dx * dx + dy * dy
    if dd <= 0:
        t = 0.0
    else:
        t = ((px - ax) * dx + (py - ay) * dy) / dd
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
    qx = ax + t * dx
    qy = ay + t * dy
    return math.sqrt((px - qx) * (px - qx) + (py - qy) * (py - qy))


def dist_to_rect_edge(px, py):
    """点到"检出的矩形边线"的最短距离(像素) = 光斑离胶带多远 —— 判分量的就是它

    🔧 为什么不按路径点索引比距离(旧写法): 2026-09-30 实测角点排序帧间会**翻** ——
    sort_corners_clockwise 取 atan2 最小的角当0号, 角点一抖 0号就换人, 于是
    path[pidx] 跳到另一个角上, 量出来的 err 能到124px=37.6cm 的**假值**(A4对角线才114px).
    最短距离跟索引无关, 怎么翻都是同一个数.
    Returns: 距离(px); 没检出矩形返回 -1
    """
    pts = rect_coords
    if len(pts) != 4:
        return -1
    best = -1.0
    for i in range(4):
        a = pts[i]
        b = pts[(i + 1) % 4]
        d = _dist_to_seg(px, py, a[0], a[1], b[0], b[1])
        if best < 0 or d < best:
            best = d
    return best


def get_rect_center():
    """获取矩形中心像素 = 两条对角线交点
    rect_coords已按顺时针排序, 对角点为(0,2)/(1,3)
    斜拍透视畸变下, 对角线交点才是正方形真实中心的投影(四角平均会偏)
    Returns:
        (cx, cy) 或 [] (未检测到时)
    """
    if len(rect_coords) != 4:
        return []
    return _line_intersection(rect_coords[0], rect_coords[2],
                              rect_coords[1], rect_coords[3])


def get_path_points():
    """获取路径点列表"""
    return path_points


def get_path_idx():
    """获取当前路径点索引"""
    return path_idx


def set_path_idx(idx):
    """设置当前路径点索引"""
    global path_idx
    path_idx = idx


def reset_detection():
    """重置检测结果"""
    global rect_coords, red_coords, path_points, path_idx, last_rect, hold_count, spot_area
    rect_coords = []
    red_coords = []
    path_points = []
    path_idx = 0
    last_rect = []
    hold_count = 0
    detect_debug["hold"] = 0
    spot_area = 0
    spot_hist[:] = []
