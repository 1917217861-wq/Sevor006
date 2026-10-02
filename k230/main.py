"""
K230 主入口 - 2023电赛E题 运动目标控制与自动追踪系统
方案A: K230直连F32C控电机, STM32只做按键+激光

架构:
  STM32 --(USART2, BB type data BCC 55)--> K230 --(UART3, 7A...7B)--> F32C --> 电机
  K230 摄像头 → 视觉检测(矩形/激光点) → 轨迹执行 → F32C电机控制

硬件:
  K230 亚博智能 豪华版 (CanMV, 带LCD触屏)
  F32C 无刷电机云台 (X轴ID=1, Y轴ID=2)
  STM32F103C8 (做人机交互, 激光PB6)

UART引脚 (亚博K230已确认):
  UART3 pin32(TX)/pin33(RX) → STM32
  UART1 pin9(TX)/pin10(RX) → F32C

视觉代码来源: 开源项目 kaka12331/2023-dian-sai-visual-gimbal
"""
import time, utime, os, sys, math, struct
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA, TOUCH, UART
import config
import vision
from f32c import F32C, F32C_AXIS_X, F32C_AXIS_Y, MODE_MULTI_TURN_T
from stm32_comm import STM32Comm
from trajectory import Trajectory

# ============================================================
# 全局对象
# ============================================================
f32c = None          # F32C 协议层实例
stm32_comm = None    # STM32 通信层实例
trajectory = None    # 轨迹执行器实例
sensor = None        # Sensor 实例 (标定采样要自己抓帧, main() 里赋值)

# ============================================================
# 硬件初始化 (移植自开源项目 + 方案A双UART)
# ============================================================
def hardware_init():
    """初始化 K230 硬件: FPIOA + Sensor + Display + Touch + 2路UART"""
    global f32c, stm32_comm, trajectory

    fpioa = FPIOA()
    # UART3: K230 ↔ STM32 (pin32=TXD, pin33=RXD)
    fpioa.set_function(config.STM32_UART_TX_PIN, FPIOA.UART3_TXD)   # pin32
    fpioa.set_function(config.STM32_UART_RX_PIN, FPIOA.UART3_RXD)   # pin33
    # UART1: K230 → F32C (pin9=TXD, pin10=RXD)
    fpioa.set_function(config.F32C_UART_TX_PIN, FPIOA.UART1_TXD)   # pin9
    fpioa.set_function(config.F32C_UART_RX_PIN, FPIOA.UART1_RXD)   # pin10

    # Sensor (🔴 SENSOR_ID 亚博K230到货后实测确认, 当前默认2)
    sensor = Sensor(id=config.SENSOR_ID)
    sensor.reset()
    sensor.set_framesize(width=config.LCD_W, height=config.LCD_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    # Display (亚博屏=ST7701 640x480, 实测800x480会竖条纹)
    display_type = getattr(Display, config.DISPLAY_TYPE, Display.ST7701)
    if config.DISPLAY_ROTATION:
        rot_flag = getattr(Display, "FLAG_ROTATION_{}".format(config.DISPLAY_ROTATION))
        Display.init(display_type, width=config.DISP_W, height=config.DISP_H,
                     to_ide=False, flag=rot_flag)
    else:
        Display.init(display_type, width=config.DISP_W, height=config.DISP_H, to_ide=False)
    MediaManager.init()
    sensor.run()

    # Touch
    tp = TOUCH(0)

    # UART3: K230 ↔ STM32
    stm32_uart = UART(config.STM32_UART_ID, baudrate=config.STM32_UART_BAUDRATE)

    # UART1: K230 → F32C
    f32c_uart = UART(config.F32C_UART_ID, baudrate=config.F32C_BAUDRATE)

    print("[K230] UART3(STM32) OK, UART1(F32C) OK")

    # 模块初始化
    f32c = F32C(f32c_uart)
    stm32_comm = STM32Comm(stm32_uart)
    trajectory = Trajectory(f32c)

    return sensor, tp


# ============================================================
# UI 绘制 (移植自开源项目)
# ============================================================
img_buf = None  # 合成缓冲区, 在main()中初始化

# 布局: 摄像头640x360占上部, 底部360~480为按钮条(640宽分4格160)
BTN_Y, BTN_H = config.LCD_H, config.DISP_H - config.LCD_H   # 360, 120

GAIN_STEP = 0.05   # 检测页 gain± 按钮每下的步长

def _btn(x, y, w, h, txt, color):
    """在(x,y)处绘制w*h填充按钮, 文字居中"""
    img_buf.draw_rectangle(x, y, w, h, color=color, fill=True)
    tw = len(txt) * 30
    img_buf.draw_string_advanced(x + (w - tw) // 2, y + (h - 30) // 2,
                                 30, txt, color=(0, 0, 0))

def ui_main():
    """主菜单: 底部5格 检测/标定/调参/LAB/复位 (每格128宽)"""
    _btn(0,   BTN_Y, 128, BTN_H, "检测", (0, 255, 0))
    _btn(128, BTN_Y, 128, BTN_H, "标定", (255, 128, 0))
    _btn(256, BTN_Y, 128, BTN_H, "调参", (255, 0, 0))
    _btn(384, BTN_Y, 128, BTN_H, "LAB",  (0, 0, 255))
    _btn(512, BTN_Y, 128, BTN_H, "复位", (0, 255, 255))

def ui_detect():
    """检测模式: 底部左两格调跟随增益(可边跑边看), 最右格退出
    🔧 为什么把增益做成按钮: 调"激光抖不抖"必须边跑边试, 而改 config.py 要推板+断电,
       一轮十几分钟. 做成按钮后现场一分钟能扫完 0~0.4, 而且能一键设成 **0** ——
       那是判断"抖是控制问题还是机械问题"的决定性实验(增益0=纯前馈, 还抖就是机械)."""
    _btn(0,   BTN_Y, 240, BTN_H, "gain-", (128, 128, 0))
    _btn(240, BTN_Y, 240, BTN_H, "gain+", (128, 128, 0))
    _btn(480, BTN_Y, 160, BTN_H, "退出", (0, 255, 255))

def ui_threshold():
    """阈值调节: 摄像头右侧竖排L调节, 底部3格 roi_s-/roi_s+/退出"""
    _btn(480, 0,   160, 88, "L_min+", (0, 255, 0))
    _btn(480, 88,  160, 88, "L_min-", (255, 0, 0))
    _btn(480, 176, 160, 88, "L_max+", (0, 0, 255))
    _btn(480, 264, 160, 88, "L_max-", (0, 255, 255))
    _btn(0,   BTN_Y, 240, BTN_H, "roi_s-", (0, 255, 0))
    _btn(240, BTN_Y, 240, BTN_H, "roi_s+", (0, 0, 255))
    _btn(480, BTN_Y, 160, BTN_H, "退出",  (128, 128, 128))
    # roi_s当前值显示在摄像头区左下角
    img_buf.draw_string_advanced(10, config.LCD_H - 34, 24,
        "roi_s={:.2f}".format(config.VISION_T[5]), (255, 255, 0))

def ui_lab_adjust():
    """LAB调节(绿色激光): 右侧竖排调 L, 底部调 A, 最后一格退出
    🔴 2026-09-30 修: 原来底部 +/- 调的是 T[6](红色激光的L_min), 而本页显示的是
       绿色掩膜 T[12..17] —— 按了看着没反应, 等于没法标定绿色. 现在四个量都能调.
    """
    _btn(480, 0,   160, 88, "L_min+", (0, 255, 0))
    _btn(480, 88,  160, 88, "L_min-", (255, 0, 0))
    _btn(480, 176, 160, 88, "L_max+", (0, 0, 255))
    _btn(480, 264, 160, 88, "L_max-", (0, 255, 255))
    _btn(0,   BTN_Y, 128, BTN_H, "A_min+", (0, 255, 0))
    _btn(128, BTN_Y, 128, BTN_H, "A_min-", (255, 0, 0))
    _btn(256, BTN_Y, 128, BTN_H, "A_max+", (0, 0, 255))
    _btn(384, BTN_Y, 128, BTN_H, "A_max-", (0, 255, 255))
    _btn(512, BTN_Y, 128, BTN_H, "退出",   (128, 128, 128))

def ui_calib():
    """标定页: 右侧竖排X/Y ±1度微调, 底部9格 ±5度/回中/设零点/测PIX2DEG/退出"""
    # 右侧竖排 (x>=480, 4格x88) 单点微调
    _btn(480, 0,   160, 88, "X+1", (0, 255, 0))
    _btn(480, 88,  160, 88, "X-1", (255, 0, 0))
    _btn(480, 176, 160, 88, "Y+1", (0, 200, 0))
    _btn(480, 264, 160, 88, "Y-1", (200, 0, 0))
    # 底部9格 (71宽)
    _btn(0,   BTN_Y, 71, BTN_H, "X+5", (0, 255, 0))
    _btn(71,  BTN_Y, 71, BTN_H, "X-5", (255, 0, 0))
    _btn(142, BTN_Y, 71, BTN_H, "Y+5", (0, 200, 0))
    _btn(213, BTN_Y, 71, BTN_H, "Y-5", (200, 0, 0))
    _btn(284, BTN_Y, 71, BTN_H, "回中", (0, 255, 255))
    _btn(355, BTN_Y, 71, BTN_H, "设零点", (255, 0, 255))
    _btn(426, BTN_Y, 71, BTN_H, "测X", (255, 255, 0))
    _btn(497, BTN_Y, 71, BTN_H, "测Y", (255, 255, 0))
    _btn(568, BTN_Y, 72, BTN_H, "退出", (128, 128, 128))


# ============================================================
# 触摸分发 (移植自开源项目 + 方案A适配)
# ============================================================
# FLAG 状态机
#   flag=0  主菜单
#   flag=1  检测模式 (视觉检测+轨迹执行)
#   flag=2  阈值调节
#   flag=3  LAB阈值调节
#   flag=4  标定辅助页 (触摸微调电机角度+读矩形中心/光斑像素)
flag = 0
last_touch = 0

# ============================================================
# 标定页状态 (flag=4)
# ============================================================
# cal_tx/cal_ty: 当前发给电机的指令角(度), 进标定页时初始化为config的HOME角
# 位置模式是绝对的, 指令角即实际停靠角, 对准幕布中心后读数就是HOME_ANGLE
cal_tx = 0.0
cal_ty = 0.0
cal_meas = None      # 最近一次测PIX2DEG的结果文本, 标定页显示

# 滚动中位数: 标定时要读的是"稳定值", 而瞬时像素帧间会抖几个像素
AVG_N = 20           # 参与中位数的帧数 (18fps下约1.1秒)
AVG_MISS_CLEAR = 15  # 连续丢这么多帧就清空, 免得拿着3秒前的旧值当现在

class RollingPoint:
    """最近N帧像素的滚动中位数。标定读数专用——瞬时值没法读。
    用中位数不用均值: 偶尔一帧误检(比如把反光当激光)不会把结果带偏。
    """
    def __init__(self, n=AVG_N, miss_clear=AVG_MISS_CLEAR):
        self.n = n
        self.miss_clear = miss_clear
        self.buf = []
        self.miss = 0

    def push(self, pt):
        """塞一帧检测结果; pt 为 None 表示本帧没检测到"""
        if pt:
            self.buf.append((pt[0], pt[1]))
            if len(self.buf) > self.n:
                self.buf.pop(0)
            self.miss = 0
        else:
            self.miss += 1
            if self.miss >= self.miss_clear:
                self.buf = []

    def get(self):
        """返回中位数像素(x,y); 无有效数据返回 None"""
        if not self.buf:
            return None
        xs = sorted(p[0] for p in self.buf)
        ys = sorted(p[1] for p in self.buf)
        m = len(xs) // 2
        return (xs[m], ys[m])

    def clear(self):
        self.buf = []
        self.miss = 0

roll_ctr = RollingPoint()   # 矩形中心
roll_red = RollingPoint()   # 激光光斑


def _fmt_angle(v):
    """安全格式化角度: 0.5度精度, 用整数拼字符串(MicroPython的float format曾崩屏)"""
    t = abs(int(round(v * 10)))
    s = "-" if v < 0 else ""
    return "%s%d.%d" % (s, t // 10, t % 10)


# ---- 测 PIX2DEG (度/像素) ----
# 粗对准公式 angle = HOME + (px-CAM_HOME)*PIX2DEG*SPOT_DIR 里,
# HOME_ANGLE / CAM_HOME 是"平移", 在标定页微调对准就能读出来;
# 但 PIX2DEG 是"缩放", 一个锚点定不了它 —— 必须靠"转已知角度、量光斑移动多少像素"测。
CAL_SWING = 5.0        # 测PIX2DEG时单侧摆动角度(度), 双侧共 2*CAL_SWING
CAL_SETTLE_MS = 800    # 摆动后等电机停稳的时间 (T型轨迹有加减速)
CAL_SAMPLE_N = 15      # 每侧采几帧取中位数

# "设零点"自动对中的参数 (原来要求人工对中, 现场漏做导致零点偏25cm, 见 calib_zero)
CAL_ZERO_ITER = 4          # 最多迭代几次把光斑压到靶心
CAL_ZERO_TOL_PX = 3        # 到位判据(px); 一次修正≈按比例消掉误差, 2~3次就能到
CAL_ZERO_SETTLE_MS = 700   # 每次修正后等电机停稳再采下一轮

def _sample_spot(n=CAL_SAMPLE_N):
    """连采n帧, 返回光斑像素中位数(x,y); 有效帧不到一半返回None"""
    xs, ys = [], []
    for _ in range(n):
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        vision.run_detect(img)
        red = vision.get_red_coords()
        if red:
            xs.append(red[0])
            ys.append(red[1])
        del img
        time.sleep_ms(10)
    if len(xs) * 2 < n:
        return None
    xs.sort()
    ys.sort()
    return (xs[len(xs) // 2], ys[len(ys) // 2])

def calib_measure_pix2deg(axis):
    """测某个轴的 PIX2DEG: 以当前角为中心 ±CAL_SWING 度摆一次, 两侧各采n帧取中位数,
       用 总角度 / 像素总位移 算换算系数。测完回起始角, 结果存进 cal_meas。

       注意: 会阻塞约4秒(等电机停稳+采样), 期间屏幕不动属正常。
    """
    global cal_tx, cal_ty, cal_meas
    name = "X" if axis == F32C_AXIS_X else "Y"
    base_x, base_y = cal_tx, cal_ty

    if axis == F32C_AXIS_X:
        plus, minus = base_x + CAL_SWING, base_x - CAL_SWING
    else:
        plus, minus = base_y + CAL_SWING, base_y - CAL_SWING

    cal_meas = "%s: measuring..." % name

    if axis == F32C_AXIS_X:
        f32c.set_position(F32C_AXIS_X, plus)
    else:
        f32c.set_position(F32C_AXIS_Y, plus)
    time.sleep_ms(CAL_SETTLE_MS)
    p_plus = _sample_spot()

    if axis == F32C_AXIS_X:
        f32c.set_position(F32C_AXIS_X, minus)
    else:
        f32c.set_position(F32C_AXIS_Y, minus)
    time.sleep_ms(CAL_SETTLE_MS)
    p_minus = _sample_spot()

    # 不管测成没测成, 先回起始角
    cal_tx, cal_ty = base_x, base_y
    f32c.set_position(F32C_AXIS_X, base_x)
    f32c.set_position(F32C_AXIS_Y, base_y)

    if p_plus is None or p_minus is None:
        cal_meas = "%s: no laser" % name
        print("[CALIB] %s failed: laser not seen" % name)
        return

    dx = p_plus[0] - p_minus[0]
    dy = p_plus[1] - p_minus[1]
    span = abs(dx) if abs(dx) >= abs(dy) else abs(dy)
    if span < 5:
        cal_meas = "%s: moved %dpx only" % (name, span)
        print("[CALIB] %s failed: span=%dpx too small" % (name, span))
        return

    p2d = (CAL_SWING * 2) / span
    # dx/dy 都打出来: 正常主轴位移应远大于副轴; 若反了说明轴装反或画面转了90度
    cal_meas = "%s PIX2DEG=%.4f dx=%d dy=%d" % (name, p2d, dx, dy)
    print("[CALIB] %s PIX2DEG=%.4f (dx=%d dy=%d span=%d)" % (name, p2d, dx, dy, span))


def calib_enter():
    """进入标定页: 只使能电机+设模式, 不移动 (零点不持久, 发绝对角会让云台乱跑)"""
    global cal_tx, cal_ty, cal_meas
    # 防御: 轨迹在跑会后台发位置指令抢电机, 先停掉
    if trajectory and trajectory.running:
        trajectory.stop()
    # 清掉上一轮的读数, 免得刚进来就显示旧值
    roll_ctr.clear()
    roll_red.clear()
    cal_meas = None
    f32c.enable(F32C_AXIS_X)
    f32c.enable(F32C_AXIS_Y)
    f32c.set_mode(F32C_AXIS_X, MODE_MULTI_TURN_T)
    f32c.set_mode(F32C_AXIS_Y, MODE_MULTI_TURN_T)
    f32c.set_speed(F32C_AXIS_X, config.F32C_DEFAULT_SPEED)
    f32c.set_speed(F32C_AXIS_Y, config.F32C_DEFAULT_SPEED)
    # 🔧 这里不发位置指令: 零点掉电不保持, 发一个绝对角等于让云台乱跑.
    #    只使能+设模式, 云台停在原地; 由操作员用微调键把光斑调到方框中心后按"设零点"
    cal_tx = config.HOME_ANGLE_X
    cal_ty = config.HOME_ANGLE_Y
    print("[CALIB] enter (enabled, no move) tx={} ty={}".format(cal_tx, cal_ty))

def calib_nudge(axis, delta):
    """标定页微调: 目标角加减delta度并立即发位置指令"""
    global cal_tx, cal_ty
    if axis == F32C_AXIS_X:
        cal_tx += delta
        f32c.set_position(F32C_AXIS_X, cal_tx)
    else:
        cal_ty += delta
        f32c.set_position(F32C_AXIS_Y, cal_ty)

def calib_home():
    """标定页回中: 回到config的HOME角 (设零点后 HOME=(0,0) = 方框中心)"""
    global cal_tx, cal_ty
    cal_tx = config.HOME_ANGLE_X
    cal_ty = config.HOME_ANGLE_Y
    f32c.set_position(F32C_AXIS_X, cal_tx)
    f32c.set_position(F32C_AXIS_Y, cal_ty)

def _sample_spot_ctr(n=30):
    """连采n帧, 返回 (光斑中位数, 矩形中心中位数)

    为什么不复用 render 里的 roll_red/roll_ctr: 标定动作是**阻塞**的(在触摸回调里跑),
    阻塞期间主循环不转 -> 那两个滚动窗口不会进新数据. 所以自己抓帧自己算.
    Returns: ((sx,sy), (cx,cy)); 有一路没检到就返回 (None, None)
    """
    xs, ys, cs, cy = [], [], [], []
    for _ in range(n):
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        vision.run_detect(img)
        red = vision.get_red_coords()
        if red:
            xs.append(red[0])
            ys.append(red[1])
        c = vision.get_rect_center()
        if c:
            cs.append(c[0])
            cy.append(c[1])
        del img
        time.sleep_ms(10)

    def med(a):
        # ⚠️ 这里**故意只要求3个有效帧**(不是一半): 光斑打在布上/黑胶带上时会变暗,
        #    检出率可能掉到20% —— 那时若按"过半"判, 设零点会直接放弃对中(2026-09-30踩过).
        #    有3帧就够取中位数定中心了, 宁可对中得糙一点, 也别卡在"看不到"上.
        if len(a) < 3:
            return None
        a.sort()
        return a[len(a) // 2]

    sx, sy = med(xs), med(ys)
    cx, cyy = med(cs), med(cy)
    if sx is None or cx is None:
        return None, None
    return (sx, sy), (cx, cyy)


def calib_zero():
    """设零点: **先用视觉把光斑自动对到靶心, 再把当前位姿定义为 0 度**

    🔧 2026-09-30 改 (现场踩的坑): 原来是"操作员先用微调键把光斑手工对到靶心(dx/dy≈0),
       然后按本键". 但偏移几十px时要按十几下, 现场漏了这一步 -> **零点整体偏了25cm,
       整圈路径整体偏右**(实测标定页 dx:82 dy:-29).
       现在按一下就行: 拿当前 dx/dy 按伺服同一个公式算出需要的角度修正量发过去,
       迭代几次把光斑压到靶心, 再 clear_multi_turn.

    注意三点:
    - 对中用的公式/标定和平时跟随用的是同一套(angle -= err*PIX2DEG*SPOT_DIR),
      所以"对中准不准"和"跑起来准不准"是一致的, 不会互相矛盾
    - 清零之后必须补发一次 position(0,0): 否则位置寄存器里还留着清零前那个角度值,
      在新参考系下被当成目标 -> 云台会突然跳一下
    - **还要把 config.CAM_HOME 更新成这个 ctr** —— 零点定的是"角度原点",
      而前馈粗对准还需要一个"像素原点", 两者必须是同一个点(2026-10-01 现场踩的坑, 见下)
    """
    global cal_tx, cal_ty, cal_meas
    if trajectory and trajectory.running:
        trajectory.stop()
    cal_meas = "ZERO: centering..."
    dx = dy = 0
    ok = False
    for _ in range(CAL_ZERO_ITER):
        spot, ctr = _sample_spot_ctr()
        if spot is None or ctr is None:
            # 看不到光斑或矩形就不动也不清零: 让它继续走原来的参考系, 人工先确认
            cal_meas = "ZERO: no laser/rect!"
            print("[CALIB] zero aborted: laser or rect not visible")
            return
        dx = spot[0] - ctr[0]
        dy = spot[1] - ctr[1]
        print("[CALIB] zero iter dx={} dy={}".format(dx, dy))
        # 兜底: 偏差大到离谱(>200px≈60cm)说明视觉读错了(锁到杂块上), 这时候不能动也不能清零
        if abs(dx) + abs(dy) > 200:
            cal_meas = "ZERO: bad read {}px".format(abs(dx) + abs(dy))
            print("[CALIB] zero aborted: implausible dx/dy")
            return
        if abs(dx) <= CAL_ZERO_TOL_PX and abs(dy) <= CAL_ZERO_TOL_PX:
            ok = True
            break
        cal_tx -= dx * config.PIX2DEG_X * config.SPOT_DIR_X
        cal_ty -= dy * config.PIX2DEG_Y * config.SPOT_DIR_Y
        f32c.set_position(F32C_AXIS_X, cal_tx)
        f32c.set_position(F32C_AXIS_Y, cal_ty)
        time.sleep_ms(CAL_ZERO_SETTLE_MS)

    # 🔧 2026-10-01: 零点建立的同时, 必须把"0度对应的那个像素"也定成同一个点.
    #    为什么: 前馈粗对准用的是 config.CAM_HOME_PX/PY 当像素原点
    #    (angle = (px - CAM_HOME)*PIX2DEG*SPOT_DIR), 而上面刚刚把"当前位姿(光斑压在ctr上)"
    #    定义成 0 度 —— 那么 CAM_HOME 就必须等于这个 ctr, 否则前馈**整体**偏
    #    (ctr - CAM_HOME), 而这一点积分要一路硬拽回来.
    #    实测(2026-10-01): A4 四角粘死后 ctr 从 (273,161) 挪到 (315,191),
    #    CAM_HOME 没跟着走 -> 前馈整体偏 42/30px≈15cm -> 整圈都在跟这 15cm 打架,
    #    光斑被拽出靶纸(spot 60%), maxerr 154px(46cm) —— 而同一份代码昨天是 10px.
    #    改成"设零点顺便定原点"之后: 纸怎么挪、相机怎么碰, 只要重新按一次设零点就自洽,
    #    不再依赖 config 里那个会过期的常量(config 值退化成"首帧兜底").
    config.CAM_HOME_PX = ctr[0]
    config.CAM_HOME_PY = ctr[1]
    print("[CALIB] CAM_HOME <- ({},{})".format(ctr[0], ctr[1]))

    f32c.clear_multi_turn(F32C_AXIS_X)
    f32c.clear_multi_turn(F32C_AXIS_Y)
    f32c.save_params(F32C_AXIS_X)
    f32c.save_params(F32C_AXIS_Y)
    cal_tx, cal_ty = 0.0, 0.0
    f32c.set_position(F32C_AXIS_X, 0.0)
    f32c.set_position(F32C_AXIS_Y, 0.0)
    cal_meas = "ZERO ok dx{} dy{} CAM({},{})".format(int(dx), int(dy), ctr[0], ctr[1]) if ok else \
        "ZERO: residual {}px CAM({},{})".format(abs(dx) + abs(dy), ctr[0], ctr[1])
    print("[CALIB] zero done ok={} residual={}px".format(ok, abs(dx) + abs(dy)))

def on_touch_main(tx, ty):
    """主菜单触摸: 底部5格 检测/标定/调参/LAB/复位"""
    global flag
    if ty < BTN_Y:
        return
    if   tx < 128:    flag = 1   # 检测
    elif tx < 256:                  # 标定
        flag = 4
        calib_enter()
    elif tx < 384:    flag = 2   # 调参
    elif tx < 512:    flag = 3   # LAB
    else:
        # 复位按钮: 直接启动复位轨迹
        if trajectory:
            trajectory.start(config.MODE_RESET)

def on_touch_detect(tx, ty):
    """检测模式触摸: 底部 gain-/gain+ 调跟随增益, 最右格退出"""
    global flag
    if ty < BTN_Y:
        return
    if tx < 240:
        config.SERVO_GAIN_LAP = max(0.0, round(config.SERVO_GAIN_LAP - GAIN_STEP, 2))
        print("[K230] lap gain -> {}".format(config.SERVO_GAIN_LAP))
    elif tx < 480:
        config.SERVO_GAIN_LAP = min(0.6, round(config.SERVO_GAIN_LAP + GAIN_STEP, 2))
        print("[K230] lap gain -> {}".format(config.SERVO_GAIN_LAP))
    else:
        flag = 0
        if trajectory and trajectory.running:
            trajectory.stop()

def on_touch_threshold(tx, ty):
    """阈值/LAB调节触摸: 退出
    阈值页(flag=2)的退出占 480..640; LAB页(flag=3)底部有5格, 退出在 512..640"""
    global flag
    if ty < BTN_Y:
        return
    if flag == 3:
        if tx > 512:
            flag = 0
    elif tx > 480:
        flag = 0

def on_touch_calib(tx, ty):
    """标定页触摸:
       右侧竖排(x>=480, y<352) X/Y ±1度;
       底部9格 X/Y ±5度 + 回中 + 设零点 + 测X/测Y(标定PIX2DEG) + 退出"""
    global flag
    # 右侧竖排 ±1度
    if tx >= 480 and ty < 352:
        if   ty < 88:   calib_nudge(F32C_AXIS_X, +1)
        elif ty < 176:  calib_nudge(F32C_AXIS_X, -1)
        elif ty < 264:  calib_nudge(F32C_AXIS_Y, +1)
        else:           calib_nudge(F32C_AXIS_Y, -1)
        return
    # 底部按钮条 (9格, 每格71宽)
    if ty >= BTN_Y:
        if   tx < 71:   calib_nudge(F32C_AXIS_X, +5)
        elif tx < 142:  calib_nudge(F32C_AXIS_X, -5)
        elif tx < 213:  calib_nudge(F32C_AXIS_Y, +5)
        elif tx < 284:  calib_nudge(F32C_AXIS_Y, -5)
        elif tx < 355:  calib_home()
        elif tx < 426:  calib_zero()
        elif tx < 497:  calib_measure_pix2deg(F32C_AXIS_X)
        elif tx < 568:  calib_measure_pix2deg(F32C_AXIS_Y)
        else:           flag = 0   # 退出(电机保持使能停在当前角)


# ============================================================
# 主渲染 (移植自开源项目 + 方案A适配)
# ============================================================
f_cnt = 0
f_last = utime.ticks_ms()
fps = 0

# 基本2/3/4 验收读数: [本圈最大跟随误差(px), 本圈采样帧数, 其中丢光斑的帧数]
# 12.4秒一圈, 瞬时值看不出问题 —— 要记峰值和丢帧率, 而且跑完就把结果存文件
lap_stat = [0.0, 0, 0]
lap_last = None        # 刚跑完那一圈的 (idx, total, elapsed_ms, maxerr, miss, frames)
lap_was_running = False

# 🔧 误差剖面: 每个路径点的最大跟随误差 —— 把屏幕当示波器用.
#    为什么需要: 2026-09-30 盯屏幕看 12 秒根本判断不出"扭"是快抖还是慢偏, 是哪儿丢检测.
#    画成条之后, 拍一张照片就能读出整圈的误差分布.
LAP_TRACE_N = 128         # 容量(路径点124个, 留点余量)
LAP_TRACE_MAXPX = 30.0    # 满格对应的误差(px), 30px≈9cm
lap_trace = [0.0] * LAP_TRACE_N   # ⚠️ 定义别删: 2026-09-30 忘写这一行, 起跑瞬间
                                  #    render() 抛 NameError -> 屏幕黑 + UART3没释放

# 🔧 伺服误差时间迹线: 最近TRACE_N帧的"光斑 - 冻结目标"**有符号**偏差.
#    为什么要有它: 2026-09-30 实测"激光在上下走时扭来扭去" —— 但 lap_trace(按路径进度)
#    和 err(按实时检出的胶带)两个仪表都不好用: 前者看不出时间波形, 后者参照物帧间会翻.
#    这条迹线的两个量(光斑/冻结目标)都不依赖胶带检测, 所以激光真抖假抖它一定看得见:
#      曲线**来回穿0** = 振荡(伺服增益/机械stick-slip)  |  曲线**整体偏一侧** = 偏置(标定/路径错)
TRACE_N = 120        # 18fps下约8秒, 够盖住整圈里的上下走段
# 满格 ±10px: 判分要求压在胶带半宽(≈3px)以内, 所以刻度必须把 1~10px 这一段放大.
# ⚠️ 2026-09-30 教训: 原来设成 ±20px 而条高只有 18px(可画范围 ±7), 3px的误差只画成
#    1px高的齿 —— 照片里根本看不出形状, 白拍了两轮
TRACE_PX = 10.0      # 纵轴半格对应的误差(px)
tr_dx = [0] * TRACE_N
tr_dy = [0] * TRACE_N
tr_n = 0             # 已填样本数(从0长到TRACE_N)
tr_head = 0          # 环形缓冲头


def _reset_servo_trace():
    global tr_n, tr_head
    tr_n = 0
    tr_head = 0


def _push_servo_trace(dx, dy):
    global tr_n, tr_head
    tr_head = (tr_head + 1) % TRACE_N
    tr_dx[tr_head] = dx
    tr_dy[tr_head] = dy
    if tr_n < TRACE_N:
        tr_n += 1


def _draw_servo_trace():
    """两条时间迹线(上=X偏差, 下=Y偏差), 各自有0线; 纵轴±TRACE_PX满格"""
    y0 = config.LCD_H - 96
    hw = 28              # 每条迹线的高度(够高才画得出细节)
    if tr_n <= 0:
        return
    w = config.LCD_W // TRACE_N
    if w < 1:
        w = 1
    span = hw // 2 - 3
    for half, arr, col in ((0, tr_dx, (0, 255, 255)), (1, tr_dy, (255, 255, 0))):
        top = y0 + half * hw
        img_buf.draw_rectangle(0, top, config.LCD_W, hw, color=(0, 0, 0), fill=True)
        cy = top + hw // 2
        # 🔧 ±3px 参考线 = 胶带半宽. 光斑要在胶带上, 这两条线以内才对
        for lim in (3.0, -3.0):
            yy = cy - int(lim * span / TRACE_PX)
            img_buf.draw_line(0, yy, config.LCD_W - 1, yy, (160, 0, 0), 1)
        img_buf.draw_line(0, cy, config.LCD_W - 1, cy, (96, 96, 96), 1)
        for k in range(tr_n):
            i = (tr_head - tr_n + 1 + k) % TRACE_N
            v = int(arr[i] * span / TRACE_PX)
            if v > span:
                v = span
            elif v < -span:
                v = -span
            img_buf.draw_line(k * w, cy, k * w, cy - v, col, 1)


def _draw_lap_trace(total):
    """误差剖面条 (贴摄像头画面下沿)
    横轴=路径进度, 纵轴=该点的最大误差; 绿<3px(胶带半宽) 黄<6.6px(2cm) 红=超2cm
    """
    y0 = config.LCD_H - 40
    h = 34
    img_buf.draw_rectangle(0, y0, config.LCD_W, h, color=(0, 0, 0), fill=True)
    # 参考线: 3px=胶带半宽, 6.6px=基本2的2cm容差
    for px, col in ((3.0, (255, 255, 0)), (6.6, (255, 0, 0))):
        yy = y0 + h - int(px * h / LAP_TRACE_MAXPX)
        img_buf.draw_line(0, yy, config.LCD_W - 1, yy, col, 1)
    if total <= 0:
        return
    # 点数多时(开环兜底是240点)自动变窄, 免得画到屏幕外
    w = config.LCD_W // total
    if w < 1:
        w = 1
    bw = w - 1
    if bw < 1:
        bw = 1
    # ⚠️ total 可能 > LAP_TRACE_N(开环兜底240点), 必须夹住, 否则 IndexError
    n = total if total <= LAP_TRACE_N else LAP_TRACE_N
    for i in range(n):
        v = lap_trace[i]
        if v <= 0:
            continue
        bh = int(v * h / LAP_TRACE_MAXPX)
        if bh > h:
            bh = h
        if bh < 1:
            bh = 1
        c = (0, 255, 0) if v < 3.0 else ((255, 255, 0) if v < 6.6 else (255, 0, 0))
        img_buf.draw_rectangle(i * w, y0 + h - bh, bw, bh, color=c, fill=True)


def _log_lap(info):
    """把本圈结果追加到 /sdcard/lap.log
    ⚠️ 为什么存文件: 2026-09-30 实测 K230 的串口输出在 main.py 运行时不一定能实时看到
    (抓到的都是开机时压在缓冲区里的旧内容), 屏幕上的瞬读数人眼又读不准.
    存成文件事后用 REPL 读回来最可靠: open('/sdcard/lap.log').read()
    """
    try:
        line = "lap {}/{} {:.1f}s maxerr {:.1f}px {:.1f}cm spotmiss {}%\n".format(
            info[0], info[1], info[2] / 1000.0, info[3], info[3] / config.PX_PER_CM,
            info[4] * 100 // info[5] if info[5] else 0)
        with open('/sdcard/lap.log', 'a') as f:
            f.write(line)
        print("[LAPLOG]", line.strip())
    except Exception as ex:
        print("[LAPLOG] fail:", ex)

def render(img):
    """主渲染函数: flag状态机分发"""
    global f_cnt, fps, f_last, lap_stat, lap_last, lap_was_running

    # FPS 统计
    f_cnt += 1
    now = utime.ticks_ms()
    dt = utime.ticks_diff(now, f_last)
    if dt > 1000:
        fps = f_cnt * 1000 / dt
        f_cnt = 0
        f_last = now
    fps_txt = "FPS: {:.1f}".format(fps) if fps > 0 else "FPS: --"

    if flag == 0:
        # ── 主菜单 ──
        img_buf.clear(roi=(0, 0, config.DISP_W, config.DISP_H))
        img_buf.draw_image(img, 0, 0)
        ui_main()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))
        # 显示STM32模式
        mode_txt = "STM32 mode={} pause={}".format(
            stm32_comm.current_mode if stm32_comm else 0,
            stm32_comm.pause_flag if stm32_comm else 0)
        img_buf.draw_string_advanced(10, 40, 20, mode_txt, (255, 255, 0))

    elif flag == 1:
        # ── 检测模式: 视觉检测 + 轨迹执行 ──
        vision.run_detect(img)
        img_buf.clear(roi=(0, 0, config.DISP_W, config.DISP_H))
        img_buf.draw_image(img, 0, 0)
        # 矩形中心(对角线交点, 品红点)
        ctr_pt = vision.get_rect_center()
        if ctr_pt:
            img_buf.draw_circle(ctr_pt[0], ctr_pt[1], 5, (255, 0, 255), thick=2, fill=True)
        # 绘制当前路径目标点 (蓝色, 沿矩形边线前进)
        # 🔧 2026-10-01: 蓝点必须画**伺服真正的目标**(trajectory 的冻结路径), 不是 vision 的实时路径点.
        #    为什么: vision 的路径是按**实时**矩形的角点顺序每帧重建的, 而代码注释里早就写着
        #    "sort_corners_clockwise 的 0 号角**帧间会翻**" —— 路径一翻, 同一个索引就指到矩形
        #    另一头去了, 蓝点会大幅跳; 而伺服跟的一直是开跑瞬间冻结的那条路径.
        #    现场后果: 人看着"激光不跟蓝点走"(其实蓝点在乱跳, 伺服是对的), 而且矩形掉检出
        #    (rect HOLD)时 vision 的路径会变陈旧, 蓝点更离谱 —— 整个下午都被这个仪表误导.
        #    没有冻结目标时(复位/开环/未跑)退回画实时路径点
        tgt = trajectory.get_target() if trajectory else None
        if tgt:
            img_buf.draw_circle(int(tgt[0]), int(tgt[1]), 8, (0, 0, 255), thick=2, fill=True)
        else:
            path = vision.get_path_points()
            pidx = vision.get_path_idx()
            if path and pidx < len(path):
                px, py = path[pidx]
                img_buf.draw_circle(px, py, 8, (0, 0, 255), thick=2, fill=True)
        ui_detect()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))
        # 底部状态
        rect = vision.get_rect_coords()
        red = vision.get_red_coords()
        if rect:
            if vision.detect_debug["hold"]:
                # 青色=保持中的上一帧旧框 (本帧未检测到)
                img_buf.draw_string_advanced(10, config.LCD_H + 5, 20,
                    "rect HOLD(h{}): {}".format(vision.detect_debug["hold"], rect),
                    (0, 255, 255))
            else:
                img_buf.draw_string_advanced(10, config.LCD_H + 5, 20,
                    "rect OK: {}".format(rect), (0, 255, 0))
        else:
            dbg = vision.detect_debug
            img_buf.draw_string_advanced(10, config.LCD_H + 5, 20,
                "rect:-- r{} a{} f{}".format(dbg["raw"], dbg["area"], dbg["angfail"]),
                (255, 0, 0))
        # 🔧 spot = 最近60帧的光斑检出率. 伺服要连续3帧都检出才给反馈, 所以这个数一低,
        #    伺服就等于开环(靠前馈航位推算走) —— 用来分清"激光跟不上"是检测丢了还是伺服跟不上
        rate = vision.get_spot_rate()
        rate_txt = "spot {}%".format(rate) if rate >= 0 else "spot --"
        if red:
            img_buf.draw_string_advanced(10, config.LCD_H + 30, 20,
                "laser: {}  {}  blob {}px  g{:.2f} s{}".format(
                    red, rate_txt, vision.get_spot_area(),
                    config.SERVO_GAIN_LAP, config.F32C_LAP_SPEED),
                (255, 255, 0))
        else:
            img_buf.draw_string_advanced(10, config.LCD_H + 30, 20,
                "laser:--  {}".format(rate_txt), (255, 0, 0))
        # 🔧 跟随精度读数(基本3/4用): 光斑到"当前路径目标点"的距离 = 伺服跟随误差.
        #    n = 通过角度校验的候选矩形数:
        #        >=2 -> 找到了内外两侧边界, 取中线(正好压在胶带中线上) ✓
        #        1   -> 只找到一侧边界, 路径走的是胶带的某一条边(会贴边甚至脱离)
        # 🔧 跟随误差 = 光斑到**实时检出的胶带边线**的最短距离(不是到某个路径点的距离).
        #    判分量的就是"光斑离胶带多远"; 而且它跟路径点索引无关 ——
        #    实测角点排序帧间会翻, 按索引比会量出 124px=37.6cm 的假值
        e = vision.dist_to_rect_edge(red[0], red[1]) if red else -1
        if e >= 0:
            nrect = vision.detect_debug["area"] - vision.detect_debug["angfail"]
            img_buf.draw_string_advanced(10, config.LCD_H + 55, 20,
                "err:{:.0f}px {:.1f}cm  n={}".format(e, e / config.PX_PER_CM, nrect),
                (255, 255, 255))

        # 🔧 整圈进度 + 本圈最大误差(基本2/3/4): 时间推进后整圈固定12.4秒, 要一眼看出跑完没
        #    maxerr 从开跑1秒后开始累计 —— 起步那一下光斑还停在方框中心, 误差必然大, 不算数
        running_lap = trajectory is not None and trajectory.running \
            and trajectory.current_mode != config.MODE_RESET
        if running_lap:
            pr = trajectory.get_progress()
            if not lap_was_running:
                lap_stat[0] = 0.0
                lap_stat[1] = 0
                lap_stat[2] = 0
                lap_trace[:] = [0.0] * LAP_TRACE_N
                _reset_servo_trace()
            else:
                lap_stat[1] += 1
                if not red:
                    lap_stat[2] += 1
                if pr['elapsed_ms'] > 1000 and e >= 0:
                    if e > lap_stat[0]:
                        lap_stat[0] = e
                    # 🔧 误差剖面: 每个路径点记它的最大误差(跳过起步1秒的抓取段)
                    #    横轴=开跑时的路径进度(冻结的索引, 一致), 纵轴=离胶带的距离
                    ti = pr['path_idx']
                    if ti < LAP_TRACE_N and e > lap_trace[ti]:
                        lap_trace[ti] = e
            # 🔧 伺服误差迹线: 光斑减"冻结目标", 与胶带检测无关, 激光真抖一定看得见
            tgt = trajectory.get_target()
            if red and tgt:
                _push_servo_trace(red[0] - tgt[0], red[1] - tgt[1])
            lap_last = (pr['path_idx'], pr['total'], pr['elapsed_ms'],
                        lap_stat[0], lap_stat[2], lap_stat[1])
            img_buf.draw_string_advanced(10, config.LCD_H + 80, 20,
                "lap {}/{} {:.1f}s maxerr:{:.0f}px {:.1f}cm".format(
                    pr['path_idx'], pr['total'], pr['elapsed_ms'] / 1000.0,
                    lap_stat[0], lap_stat[0] / config.PX_PER_CM),
                (0, 255, 255))
            _draw_lap_trace(pr['total'])
            _draw_servo_trace()
        elif lap_was_running and lap_last is not None:
            # 🔧 刚跑完: 存文件 + 把结果留在屏幕上(数字随 running 一起消失的话, 人眼根本读不到)
            _log_lap(lap_last)
            img_buf.draw_string_advanced(10, config.LCD_H + 80, 20,
                "LAST {}/{} {:.1f}s maxerr:{:.0f}px {:.1f}cm miss{}%".format(
                    lap_last[0], lap_last[1], lap_last[2] / 1000.0,
                    lap_last[3], lap_last[3] / config.PX_PER_CM,
                    lap_last[4] * 100 // lap_last[5] if lap_last[5] else 0),
                (128, 255, 128))
            _draw_lap_trace(lap_last[1])   # 跑完把剖面留在屏幕上
            _draw_servo_trace()            # 迹线也留着(它是最后8秒的波形)
        lap_was_running = running_lap

    elif flag == 2:
        # ── 阈值调节 ──
        preview = vision.run_threshold_preview(img)
        img_buf.clear(roi=(0, 0, config.DISP_W, config.DISP_H))
        img_buf.draw_image(preview, 0, 0)
        ui_threshold()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))
        del preview

    elif flag == 3:
        # ── LAB阈值调节 ──
        preview = vision.run_lab_preview(img)
        img_buf.clear(roi=(0, 0, config.DISP_W, config.DISP_H))
        img_buf.draw_image(preview, 0, 0)
        ui_lab_adjust()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))
        del preview

    elif flag == 4:
        # ── 标定辅助页: 持续检测矩形+光斑, 读数=第一问标定的4组参数 ──
        # ctr=矩形中心像素(CAM_HOME候选), laser=光斑像素,
        # dx/dy=光斑相对中心偏差(0=对准), tx/ty=当前指令角(HOME_ANGLE候选)
        vision.run_detect(img)
        # 读数取最近20帧中位数: 瞬时值帧间抖几个像素, 直接读会把误差写进标定
        roll_ctr.push(vision.get_rect_center())
        roll_red.push(vision.get_red_coords())

        img_buf.clear(roi=(0, 0, config.DISP_W, config.DISP_H))
        img_buf.draw_image(img, 0, 0)
        ui_calib()
        img_buf.draw_string_advanced(10, 10, 24, fps_txt, (255, 255, 255))

        ctr_pt = roll_ctr.get()
        red = roll_red.get()
        if ctr_pt:
            img_buf.draw_circle(ctr_pt[0], ctr_pt[1], 5, (255, 0, 255), thick=2, fill=True)
            img_buf.draw_string_advanced(10, 38, 20,
                "ctr:(%d,%d)" % (ctr_pt[0], ctr_pt[1]), (0, 255, 255))
        else:
            img_buf.draw_string_advanced(10, 38, 20, "ctr:-- not locked", (255, 0, 0))
        if red:
            img_buf.draw_string_advanced(10, 62, 20,
                "laser:(%d,%d)" % (red[0], red[1]), (255, 255, 0))
        else:
            img_buf.draw_string_advanced(10, 62, 20, "laser:--", (255, 0, 0))
        if ctr_pt and red:
            img_buf.draw_string_advanced(10, 86, 20,
                "dx:%d dy:%d" % (red[0] - ctr_pt[0], red[1] - ctr_pt[1]),
                (255, 255, 255))
        img_buf.draw_string_advanced(10, 110, 20,
            "tx:%s ty:%s" % (_fmt_angle(cal_tx), _fmt_angle(cal_ty)),
            (0, 255, 0))

        # 矩形在画面里占多少像素 → 直接决定 1.8cm 胶带还剩几个像素可检
        rect = vision.get_rect_coords()
        if len(rect) == 4:
            rx = [p[0] for p in rect]
            ry = [p[1] for p in rect]
            w_px = max(rx) - min(rx)
            img_buf.draw_string_advanced(10, 134, 20,
                "rect %dx%d px  1.8cm tape=%.1f px" % (
                    w_px, max(ry) - min(ry), w_px * 1.8 / 50.0),
                (200, 200, 255))

        if cal_meas:
            img_buf.draw_string_advanced(10, 158, 20, cal_meas, (255, 128, 0))


# ============================================================
# STM32 指令回调
# ============================================================
def setup_stm32_callbacks():
    """注册STM32指令回调"""
    def on_mode_change(mode):
        global flag
        print("[K230] MODE: {}".format(mode))
        if mode == config.MODE_RESET:
            trajectory.start(config.MODE_RESET)
        elif mode in (config.MODE_EDGE, config.MODE_TAPE, config.MODE_TAPE_ROTATE):
            # 🔧 工作模式自动进检测模式(flag=1): 矩形边线检测+光斑识别必须持续运行,
            #    视觉伺服路径才有目标和反馈; 否则会退回开环兜底
            flag = 1
            trajectory.start(mode)
        elif mode == config.MODE_IDLE:
            if trajectory.running:
                trajectory.stop()
            # 🔧 不失能!! 失能让云台失去力矩而垂下, 而F32C会在使能时重建零点
            #    -> 每次失能都让整个像素↔角度映射偏移一点
            #    (实测2026-09-27: 每轮PB0过IDLE累积约+10px, 4轮+38px)
            #    伺服不发新目标就会自己锁住当前位置, 要停就停轨迹

    def on_pause(pause):
        print("[K230] PAUSE: {}".format(pause))
        if pause:
            # 🔧 制动 = 停轨迹(不再发新目标), 伺服自己锁住位置 -> 不使能也不失能
            #    (赛题要求暂停时立即制动; 失能让云台松掉是错的实现)
            trajectory.pause()
        else:
            # 🔧 恢复: 直接续跑, 不重新使能(使能会重建零点)
            trajectory.resume()

    def on_laser(on):
        print("[K230] LASER: {} (STM32已直接控)".format(on))

    stm32_comm.on_mode_change = on_mode_change
    stm32_comm.on_pause = on_pause
    stm32_comm.on_laser = on_laser


# ============================================================
# 阈值调节处理 (移植自开源项目)
# ============================================================
PSTEP = [1000, 500, 1000, 5, 5, 0.1, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]
PLIMIT = [(0, 30000), (0, 100000), (0, 100000), (0, 255), (0, 255),
          (0.5, 2), (0, 100), (0, 100), (-128, 127), (-128, 127), (-128, 127), (-128, 127),
          (0, 255), (0, 255), (-128, 127), (-128, 127), (-128, 127), (-128, 127)]

def handle_threshold_touch(tx, ty):
    """阈值调节触摸: 连续调节 (无防抖)"""
    T = config.VISION_T
    # L调节4键: 摄像头右侧竖排 (x>=480, y<352), 仅调参模式
    if flag == 2 and tx >= 480 and ty < 352:
        if   ty < 88:   T[3] += PSTEP[3]           # L_min+
        elif ty < 176:  T[3] -= PSTEP[3]           # L_min-
        elif ty < 264:  T[4] += PSTEP[4]           # L_max+
        elif ty < 352:  T[4] -= PSTEP[4]           # L_max-
    # LAB页(flag=3)右侧竖排: 绿色 L_min(T[12]) / L_max(T[13])
    elif flag == 3 and tx >= 480 and ty < 352:
        if   ty < 88:   T[12] += PSTEP[12]         # 绿 L_min+
        elif ty < 176:  T[12] -= PSTEP[12]         # 绿 L_min-
        elif ty < 264:  T[13] += PSTEP[13]         # 绿 L_max+
        elif ty < 352:  T[13] -= PSTEP[13]         # 绿 L_max-
    # 底部按钮条 (y>=360)
    if ty >= BTN_Y:
        if flag == 2:
            if tx < 240:    T[5] -= PSTEP[5]       # roi_s-
            elif tx < 480:  T[5] += PSTEP[5]       # roi_s+
        elif flag == 3:
            # 绿色 A_min(T[14]) / A_max(T[15]): 光斑核心过曝时A会趋近0,
            # 卡在-10就检不到; 黑胶带上又可能偏暗 -> 这两个是让检出率上100%的关键
            if   tx < 128:  T[14] += PSTEP[14]     # A_min+
            elif tx < 256:  T[14] -= PSTEP[14]     # A_min-
            elif tx < 384:  T[15] += PSTEP[15]     # A_max+
            elif tx < 512:  T[15] -= PSTEP[15]     # A_max-
    # 限幅
    if flag == 2 or flag == 3:
        for i in range(len(T)):
            lo, hi = PLIMIT[i]
            T[i] = max(lo, min(T[i], hi))


# ============================================================
# 主函数
# ============================================================
def main():
    global img_buf, flag, last_touch, sensor

    # 1. 硬件初始化
    sensor, tp = hardware_init()

    # 2. 初始化合成缓冲区
    img_buf = image.Image(config.DISP_W, config.DISP_H, image.RGB565)

    # 3. 注册STM32回调
    setup_stm32_callbacks()

    # 4. 初始化电机: 使能 + 多圈T模式 + 巡航速度
    f32c.enable(F32C_AXIS_X)
    f32c.enable(F32C_AXIS_Y)
    time.sleep(0.01)
    f32c.set_mode(F32C_AXIS_X, MODE_MULTI_TURN_T)
    f32c.set_mode(F32C_AXIS_Y, MODE_MULTI_TURN_T)
    # T型轨迹巡航速度, 不发的话用速度寄存器残留值(可能为0/极小, 实测踩过坑)
    f32c.set_speed(F32C_AXIS_X, config.F32C_DEFAULT_SPEED)
    f32c.set_speed(F32C_AXIS_Y, config.F32C_DEFAULT_SPEED)

    print("[K230] BOOT OK (方案A: K230直连F32C)")
    print("[K230] 等待STM32指令或触屏操作...")

    # 5. 主循环
    first_loop = True   # 🔍调试: 首圈阶段标记
    try:
        while True:
            os.exitpoint()
            now = utime.ticks_ms()

            # 采集图像
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            if first_loop:
                print("[DBG] 1.snapshot OK:", img.width(), "x", img.height())

            # 触摸处理
            p = tp.read()
            if first_loop:
                print("[DBG] 2.tp.read OK:", p)
            if p:
                tx, ty = p[0].x, p[0].y

                # 连续调节 (无防抖)
                handle_threshold_touch(tx, ty)

                # 状态切换 (带防抖)
                if utime.ticks_diff(now, last_touch) > config.TOUCH_DEBOUNCE_MS:
                    if   flag == 0:  on_touch_main(tx, ty)
                    elif flag == 1:  on_touch_detect(tx, ty)
                    elif flag == 2:  on_touch_threshold(tx, ty)
                    elif flag == 3:  on_touch_threshold(tx, ty)  # LAB退出
                    elif flag == 4:  on_touch_calib(tx, ty)
                    # 触摸反馈
                    img_buf.draw_cross(tx, ty, (255, 0, 0), 10, 3)
                    last_touch = now

            # 渲染
            if first_loop:
                print("[DBG] 3a.render begin")
            render(img)

            # STM32指令轮询
            if first_loop:
                print("[DBG] 3b.render OK, poll begin")
            stm32_comm.poll()

            # 轨迹执行 (检测模式或STM32触发)
            if trajectory.running:
                trajectory.step()

            # 显示
            if first_loop:
                print("[DBG] 3c.poll OK, show begin")
            Display.show_image(img_buf)

            if first_loop:
                print("[DBG] 3d.show_image OK, 进入稳态循环")
                first_loop = False

            del img
            import gc
            gc.collect()
            time.sleep_ms(5)

    except KeyboardInterrupt:
        print("[K230] 用户停止")
    except BaseException as e:
        print("[K230] 异常:", repr(e))
        try:
            import sys
            if hasattr(sys, 'print_exception'):
                sys.print_exception(e)
        except Exception:
            pass
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
