"""
轨迹执行模块 - 2023电赛E题
把视觉检测的路径点(屏幕坐标)转换为电机角度, 控制F32C执行轨迹

4种轨迹模式:
- MODE_RESET: 复位 (激光回屏幕原点, 误差≤2cm)
- MODE_EDGE: 沿屏幕0.5m正方形边线一周 (30s内, 距边线≤2cm)
- MODE_TAPE: 沿A4胶带一周 (30s内)
- MODE_TAPE_ROTATE: 沿旋转A4胶带一周

屏幕坐标→电机角度映射 (2026-09-07 K230改为固定外部机位, eye-to-hand):
- 粗对准: angle = HOME_ANGLE + (pixel - CAM_HOME) * PIX2DEG * SPOT_DIR
  (屏幕中心像素 ↔ HOME电机角, 现场标定, 参数在config.py "固定机位视觉映射参数")
- 视觉伺服: 光斑像素误差 → 电机角修正 (增益SERVO_GAIN / SERVO_GAIN_LAP)
- 光斑不可见时自动退化为纯粗对准, 不会卡死

推进方式:
- 复位(MODE_RESET): "到位才停"式, 按PATH_INTERVAL_MS节流 + 严格容差判定
- 边线/胶带(MODE_EDGE/TAPE/TAPE_ROTATE): 按时间推进(idx = elapsed/LAP_INTERVAL_MS),
  整圈时间与帧率、跟随好坏都无关 —— 详见 _step_lap
"""
import time
import config
import vision
from f32c import F32C, F32C_AXIS_X, F32C_AXIS_Y

# === 标定参数 (在config.py "固定机位视觉映射参数"块, 此处不再重复定义) ===
# 屏幕中心像素CAM_HOME_PX/PY ↔ 电机角HOME_ANGLE_X/Y, 像素→度PIX2DEG_X/Y
# 光斑方向SPOT_DIR_X/Y, 伺服增益SERVO_GAIN / SERVO_GAIN_LAP

# 🔧 复位/开环轨迹的基准点 = 激光指向屏幕中心的HOME角度 (与config标定联动)
RESET_TARGET_X = config.HOME_ANGLE_X   # X轴复位角(度)
RESET_TARGET_Y = config.HOME_ANGLE_Y   # Y轴复位角(度)

# 🔧 复位到位容差: 基本1要求光斑距原点≤2cm(≈6~7px).
#    容差=伺服"停手"的门槛, 直接决定残差上限.
#    实测: 6px时残差卡在4~6px; 4px时y仍有+3.5px偏置(光斑太小, 质心噪声让伺服"刚进容差就停")
#    -> 收到2px: 让它基本走满3秒超时, 用积分把噪声平均掉(均值收敛到0), 而不是刚够就停
#    ⚠️ 代价: 复位要2~3秒. 基本1不限时, 值得
RESET_TOLERANCE_PX  = 2

# 🔧 伺服用光斑读数取最近N次采样的中位数: 单帧检测帧间会跳几个像素,
#    直接用瞬时值会让指令在目标附近来回抖(实测落点散布±10px)
SERVO_MED_N = 3
# 🔧 连续N次都在容差内才算到位: 伺服收敛时会"路过"目标点, 单次命中就停会停错地方
ARRIVE_HITS_NEED = 3

# 🔧 anti-windup: **目标突跳后, 暂停积分 N 帧**, 让前馈自己把云台搬过去.
#    现象(要治的): 起跑那一两秒光斑"弹一下"(先冲过头再退回来), 剖面最左边一簇红柱.
#    机理: 起步时光斑还在A4中心, 目标已经跳到路径起点(一跳几十px), 而积分是每帧累加
#          0.3×误差 -> 飞行几帧就冲出一个远超需要的修正量 -> 云台冲过头再退回来.
#
#    ★判据必须是"**目标突跳**", 不能是"误差大" —— 2026-09-30 我前后用
#      "误差>30px就不积分" 和 "修正量封顶±3°" 各试了一次, **两次都把大偏差永久锁死**:
#      实测现象是"蓝点(目标)在胶带上, 绿十字(激光)离它51px再也不动" —— 因为
#      标定/零点造成的几十px偏差永远落在门限外, 永远不被修正. 目标突跳就干净得多:
#      正常跟踪时目标每帧只走1~2px, 只有起步/换路径才会跳几十px, 两者绝不会混.
TRACK_JUMP_PX = 15           # 目标帧间跳多少px算"突跳"(正常跟踪约1~2px/帧)
TRACK_JUMP_HOLD_FRAMES = 10  # 突跳后暂停积分多少帧(18fps下约0.55秒)

# 🔧 积分死区(px, 每轴): 误差小于它就不再积分.
#    现象(2026-09-30): "起步不是迅速定位, 而是左右扭" —— 光斑质心每帧抖1~4px, 而积分是
#    每帧累加0.3×误差, 误差一小就变成**指令的随机游走**, 水平轴(X, 惯量最大)最明显.
#    死区把"已经够准了"的情况按住不动, 指令就不再抖.
#    ⚠️ 只挡**小**误差(≤2px), 大的标定偏差照常积分补回来 —— 和上面那条教训不冲突.
TRACK_DEADBAND_PX = 2

# 🔧 每帧修正量限幅(度/帧) —— 2026-10-01 加.
#    现场现象: 激光"走着走着抽一下、被抽出靶纸", 而且**增益越大抽得越狠**
#    (g0.10 偶尔抽一下; g0.35/0.60 抽到飞出白屏) —— 但**抽完能自己回到路径上**.
#    这会自己回来的特征, 说明不是环路发散(发散不会自愈), 而是"某几帧的读数坏了 × 增益"被放大
#    (正是 config 里记过的"误检有害: 误检会把指令猛拽一下").
#    限幅把"一帧最多修多少度"钉死: 坏读数/大偏差都只能一帧修一点点, 抽的幅度就有了上限.
#    ⚠️ 与 2026-09-30 踩的坑区分: 那次封的是**累积量**(修正量±3°), 会把大偏差**永久锁死**;
#       这里封的是**每帧增量**, 累积量不封顶 —— 偏差再大也能一路补回来, 只是慢一点.
#    ⚠️ 取值要**同时**照顾两件事(2026-10-01 现场吃过的亏):
#       - 抗坏读数/抗猛拽: 越小越安全(一帧最多走这么远)
#       - 补静态偏差的速度: 越大越快. 偏 130px 时, 0.5°/帧 要 3 秒才追上 ——
#         而一整圈才 12.4 秒, 于是"前半段压根跟不上"(现场实测就是这么表现的)
#       1.0°/帧 ≈ 6.4px/帧: 坏读数最多拽 6px(不限幅时能拽 78px), 130px 的偏差 1.4 秒补完
CORR_STEP_MAX_DEG = 1.0     # 1.0°/帧 ≈ 6.4px/帧 ≈ 2cm/帧 @1m

# 🔧 光斑读数跳变门控(px) —— 2026-10-01 加, **这才是"抽"的治本**.
#    现场证据(/sdcard/traj.log, g0.00 那几圈 corr 恒为0): 真读数下 |光斑-目标| 只有 5~6px,
#    但读数会**突然跳到画面右上某个固定位置**(x≈410~440, y≈55~90), err 瞬间 ~150px ——
#    这是**误检(假块)**, 而 find_blobs 取的是最大块.
#    有增益时伺服就朝那个假位置猛拽(增益越大拽得越狠, 与现场观察一致), 抽完真读数回来又自己收敛.
#    门控: 光斑在画面里逐帧最多走几 px(限幅后 ≤6px/帧), 所以**与上一次被采纳的读数差太远的
#    一定是坏读数**, 直接当"这一帧没看见"处理(不积分, 前馈照常推) —— 安全, 因为漏检无害.
#    ⚠️ 必须留逃生口: 起步/前置对位时前馈会把激光整体搬一次(一次跳上百px), 连拒几次之后
#       强行采纳一次, 免得真光斑被永久拒绝(那就成了"瞎着开环")
SPOT_GATE_PX = 30           # 与上次采纳读数差超过它就判为坏读数
SPOT_GATE_MAX_REJECT = 5    # 连续拒这么多次后强行采纳(逃生口)

# 🔧 离群记录: |光斑-目标| 超过它就记一行到 /sdcard/traj.log(跑完一圈时落盘).
#    为什么: 串口在 main.py 运行时看不到实时 print, 屏幕上又是一瞬 —— 要把"抽那一下"的
#    原始数据留下来, 才分得清"单帧坏读数"和"偏差被慢慢拽大".
SERVO_OUTLIER_PX = 30
SERVO_OUTLIER_MAX = 60      # 最多记这么多行, 防文件爆掉

# 🔧 整圈开始前的"前置对位"阶段: 先把光斑搬到起点、等它贴上去, **然后才开始计时**
#    不做这一步(2026-09-30 现场现象): 起跑那一两秒光斑还在从A4中心往路径上飞, 这段路
#    正好压在**起点那条边**上 -> "其余三条边正常, 起点那条边在乱扭", 而且那1~2秒还被
#    算进了整圈时间. 做了之后整圈12.4秒全部是"已经贴在胶带上"的行走.
LAP_SETTLE_TOL_PX = 4        # 光斑离起点这么近就算对位完成
LAP_SETTLE_TIMEOUT_MS = 2000  # 最多等这么久(看不到光斑时不能死等)
# 🔧 还要**连续这么多帧**都在容差内才算对位完成:
#    扭动的过程中误差会**瞬时**压进容差, 单帧判定会让对位在"正好路过"时就结束,
#    于是"蓝点已经走了一段, 激光还在扭"(2026-09-30 现场)
LAP_SETTLE_HITS = 3

# 🔧 开环兜底轨迹 (仅矩形持续锁不到时使用; 正常走视觉伺服路径, 见start())
# 半幅±10度: 按PIX2DEG_X=0.085估算, 0.5m正方形在~1.1m距离对应半幅约12度, 取10保守不出屏
# 8个角点, 按LAP_INTERVAL_MS=100ms计算, 240点≈24秒
OPEN_LOOP_EDGE = [
    (RESET_TARGET_X, RESET_TARGET_Y),           # 起点: 屏幕中心
    (RESET_TARGET_X + 10, RESET_TARGET_Y),      # 右边中点
    (RESET_TARGET_X + 10, RESET_TARGET_Y - 10), # 右上角
    (RESET_TARGET_X - 10, RESET_TARGET_Y - 10), # 左上角
    (RESET_TARGET_X - 10, RESET_TARGET_Y + 10), # 左下角
    (RESET_TARGET_X + 10, RESET_TARGET_Y + 10), # 右下角
    (RESET_TARGET_X + 10, RESET_TARGET_Y),      # 回右边
    (RESET_TARGET_X, RESET_TARGET_Y),           # 回中心
]

# 🔧 视觉不可用时超时自动完成 (毫秒)
ARRIVE_TIMEOUT_MS = 3000   # 3秒没检测到激光点就认为到位

# 🔧 启动后等视觉锁定矩形的最长时间 (毫秒), 超时才用开环兜底
VISION_LOCK_TIMEOUT_MS = 3000


def _clamp_step(d):
    """把单帧修正量夹在 ±CORR_STEP_MAX_DEG 内(累积量不夹, 见常量说明)"""
    if d > CORR_STEP_MAX_DEG:
        return CORR_STEP_MAX_DEG
    if d < -CORR_STEP_MAX_DEG:
        return -CORR_STEP_MAX_DEG
    return d


def _interpolate(points, total):
    """把少量角点插值成密集路径点
    Args:
        points: [(x,y), ...] 角点列表
        total: 总点数
    Returns:
        [(x,y), ...] 插值后的密集点
    """
    if len(points) < 2:
        return list(points) * total
    result = []
    seg = len(points) - 1
    per_seg = max(1, total // seg)
    for i in range(seg):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        for j in range(per_seg):
            t = j / per_seg
            result.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    # 补到total
    while len(result) < total:
        result.append(points[-1])
    return result[:total]


class Trajectory:
    """轨迹执行器"""

    def __init__(self, f32c):
        """初始化
        Args:
            f32c: F32C 云台控制对象
        """
        self.f32c = f32c
        self.current_mode = config.MODE_IDLE
        self.path_points = []
        self.path_idx = 0
        self.last_move_time = 0
        self.start_time = 0
        self.running = False
        self.paused_mode = config.MODE_IDLE   # 🔧 暂停时保存的mode, 用于恢复
        self.angle_mode = False   # 🔧 True=路径点是电机角度(开环), False=像素坐标(视觉)
        self.arrive_start_time = 0  # 🔧 到位等待超时计时
        self.pending_start = False  # 🔧 True=已启动但矩形未锁定, step里等视觉路径
        self._servo_target = None   # 🔧 积分伺服: 当前目标像素
        self._servo_angle_x = 0.0   # 🔧 积分伺服: 累积的指令角(目标不变时在此之上继续修正)
        self._servo_angle_y = 0.0
        self._spot_buf = []         # 🔧 伺服用光斑读数缓存(取中位数用)
        self._arrive_hits = 0       # 🔧 连续到位次数(去抖)
        self._track_corr_x = 0.0    # 🔧 lap伺服: 只累积视觉残差的修正量(度), 粗对准另算
        self._track_corr_y = 0.0
        self._hold_frames = 0       # 🔧 anti-windup: 还要再跳过多少帧的积分
        self._last_tgt = None       # 🔧 上一帧的目标(用来发现"目标突跳")
        self._start_idx = 0         # 🔧 整圈起点(离起跑时光斑最近的路径点)
        self._settling = False      # 🔧 True=正在前置对位, 还没开始计时
        self._settle_hits = 0       # 🔧 对位去抖: 连续在容差内的帧数
        self._cur_target = None     # 🔧 本帧的实际目标像素(给屏幕迹线当参照, 与指令用同一个值)
        self._outliers = []         # 🔧 离群记录缓冲(|光斑-目标|超门限时记一行, 见 _log_outlier)
        self._last_spot = None      # 🔧 上一次**被采纳**的光斑读数(跳变门控的参照)
        self._gate_rej = 0          # 🔧 连续被门控拒掉的次数(到 SPOT_GATE_MAX_REJECT 放行一次)

    def start(self, mode):
        """启动轨迹模式
        Args:
            mode: config.MODE_RESET / MODE_EDGE / MODE_TAPE / MODE_TAPE_ROTATE
        """
        self.current_mode = mode
        self.path_idx = 0
        self.last_move_time = time.ticks_ms()
        self.start_time = time.ticks_ms()
        self.running = True
        # 🔧 本次顺手补的既有坑(不是本次改动引入的): 原来只有边线/胶带分支会写 pending_start,
        #    复位分支不写 -> 若上一次轨迹在"等矩形锁定"时被停止(比如启动后马上按 PB0 回 IDLE),
        #    pending_start 会留成 True; 下次按复位时 step() 就会把 vision 的**胶带矩形路径**
        #    当成复位路径, 光斑跑到胶带角上而不是方框中心. 这一行恢复"start() 初始化全部运行态"的不变量
        self.pending_start = False
        # 🔧 每次启动都从粗对准重新起头: 免得上一次累积的指令角被沿用
        #    (中途"设零点"会换掉角度参考系, 累积值就失效了)
        self._servo_target = None
        self._spot_buf = []
        self._arrive_hits = 0
        self._track_corr_x = 0.0
        self._track_corr_y = 0.0
        # 🔧 起步这一下也算"突跳": 头TRACK_JUMP_HOLD_FRAMES帧先不积分, 让前馈把云台
        #    从当前位姿搬到路径起点(否则积分冲高 -> 起步"弹一下")
        self._hold_frames = TRACK_JUMP_HOLD_FRAMES
        self._last_tgt = None
        self._cur_target = None
        self._outliers = []
        self._last_spot = None
        self._gate_rej = 0

        # 确保电机使能
        self.f32c.enable(F32C_AXIS_X)
        self.f32c.enable(F32C_AXIS_Y)

        # 设置默认加速度
        self.f32c.set_accel(F32C_AXIS_X, config.F32C_DEFAULT_ACCEL)
        self.f32c.set_accel(F32C_AXIS_Y, config.F32C_DEFAULT_ACCEL)

        # 🔧 恢复巡航速度(2026-09-30 现场踩的坑, 必加):
        #    stop()/pause() 会把速度寄存器清成0, 而位置模式是靠F32C内部的T型轨迹规划器跑的
        #    —— **速度为0时发位置指令云台也不动**. 现象就是"跑完一圈/暂停过之后再起跑,
        #    指令在发、屏幕蓝点在走、激光冻在原地". 启动时补一次速度就好了.
        #    (config里"不发的话用速度寄存器残留值(可能为0/极小, 实测踩过坑)"说的就是这个;
        #     原来只在main()开机时补过一次, 没补进start(), 所以暂停/恢复也一起中招)
        # 🔧 lap 用慢速巡航(默认60RPM比需要快85倍, 小步移动会变成"小冲小刹" = 抖);
        #    复位是大步移动, 仍用默认速度. 见 config.F32C_LAP_SPEED 的说明
        _spd = config.F32C_DEFAULT_SPEED if mode == config.MODE_RESET else config.F32C_LAP_SPEED
        self.f32c.set_speed(F32C_AXIS_X, _spd)
        self.f32c.set_speed(F32C_AXIS_Y, _spd)

        # 获取路径点
        if mode == config.MODE_RESET:
            # 🔧 复位改走视觉闭环: 靶子 = 实时读到的A4靶纸中心(≈方框中心, 实测差2px).
            #    为什么不直接用config里冻结的CAM_HOME: 场地(软布/A4)自己会漂几px
            #    (实测两次读数差了10px≈3cm > 基本1的2cm容差), 冻结靶子会把漂移全算成复位误差;
            #    用实时ctr则自动跟着漂. 检测不到矩形时退回config的CAM_HOME.
            #    光斑不可见时 _move_to_pixel 自动退化为纯粗对准, 3秒超时兜底, 不会卡死
            ctr = vision.get_rect_center()
            if ctr:
                self.path_points = [(ctr[0], ctr[1])]
            else:
                self.path_points = [(config.CAM_HOME_PX, config.CAM_HOME_PY)]
            self.angle_mode = False
        else:
            # 边线/胶带轨迹: 优先视觉路径(检测到的矩形边线像素点+视觉伺服)
            vision_points = vision.get_path_points()
            if vision_points:
                self.path_points = vision_points
                self.angle_mode = False
                self.pending_start = False
            else:
                # 🔧 矩形还没锁定: 不立刻大开环, 先pending等视觉(step里逐帧试)
                # 等VISION_LOCK_TIMEOUT_MS还锁不到才用缩小的开环兜底, 避免光斑飞出幕布
                self.path_points = []
                self.angle_mode = False
                self.pending_start = True
                print("[TRAJ] waiting for rect lock (vision path)...")

        # 🔧 起点与前置对位 (⚠️ 必须放在路径点取好之后: 放在前面会拿到上一次跑剩的旧路径)
        self._pick_start_idx()

        self.arrive_start_time = time.ticks_ms()
        print("[TRAJ] start mode={} points={} angle_mode={}".format(
            mode, len(self.path_points), self.angle_mode))

    def stop(self):
        """停止轨迹"""
        self.running = False
        self.f32c.set_speed(F32C_AXIS_X, 0)
        self.f32c.set_speed(F32C_AXIS_Y, 0)
        print("[TRAJ] stop")

    def pause(self):
        """暂停轨迹 (保存当前mode, 恢复时用)"""
        self.paused_mode = self.current_mode
        self.running = False
        self.f32c.set_speed(F32C_AXIS_X, 0)
        self.f32c.set_speed(F32C_AXIS_Y, 0)
        print("[TRAJ] pause (mode={})".format(self.paused_mode))

    def resume(self):
        """恢复轨迹 (重新启动暂停前的mode)"""
        if self.paused_mode != config.MODE_IDLE:
            print("[TRAJ] resume (mode={})".format(self.paused_mode))
            self.start(self.paused_mode)
        else:
            print("[TRAJ] resume: no saved mode")

    def step(self):
        """主循环调用: 执行轨迹步进
        Returns:
            True=轨迹完成, False=进行中/未运行
        """
        if not self.running:
            return False

        now = time.ticks_ms()

        # 超时检查 (30s限制)
        if self.current_mode != config.MODE_RESET:
            elapsed = time.ticks_diff(now, self.start_time)
            if elapsed > 30000:
                print("[TRAJ] timeout at {}ms".format(elapsed))
                self.running = False
                return True  # 超时完成

        # 🔧 pending: 启动时矩形未锁定, 逐帧等视觉路径; 超时才用开环兜底
        if self.pending_start:
            vp = vision.get_path_points()
            if vp:
                self.path_points = vp
                self.angle_mode = False
                self.pending_start = False
                self.path_idx = 0
                self.arrive_start_time = now
                # 🔧 整圈计时从"锁定到矩形"这一刻起: 等待锁定不该算进这12.4秒
                self.start_time = now
                self._pick_start_idx()   # 选起点 + 进入前置对位
                print("[TRAJ] rect locked, vision path points={}".format(len(vp)))
            elif time.ticks_diff(now, self.start_time) > VISION_LOCK_TIMEOUT_MS:
                self.path_points = _interpolate(OPEN_LOOP_EDGE, 240)
                self.angle_mode = True
                self.pending_start = False
                self.start_time = now
                self.last_move_time = now
                print("[TRAJ] no rect in 3s, open-loop fallback")
            else:
                return False  # 下一帧再试

        if not self.path_points:
            return False

        if self.current_mode == config.MODE_RESET:
            return self._step_reset(now)
        return self._step_lap(now)

    def _step_reset(self, now):
        """复位(基本1): 按PATH_INTERVAL_MS节流 + 视觉伺服 + 严格容差去抖到位. 原逻辑未改
        (这一条的参数被实测调过: 别为了别的模式去动 PATH_INTERVAL_MS / SERVO_GAIN)
        """
        if time.ticks_diff(now, self.last_move_time) < config.PATH_INTERVAL_MS:
            return False
        self.last_move_time = now
        target = self.path_points[self.path_idx]

        # 🔧 读数取滚动中位数, 伺服和到位判定共用
        spot = self._spot_median()
        self._move_to_pixel(target[0], target[1], spot)
        # 🔧 视觉模式: 同步目标点索引给vision (复位恒为0; 保持与原逻辑一致)
        vision.set_path_idx(self.path_idx)

        # 🔧 连续ARRIVE_HITS_NEED次都在容差内才算到位:
        #    伺服收敛过程中会"路过"目标点, 单次命中就当到达 -> 落点随机散布(实测±10px)
        if spot and abs(spot[0] - target[0]) + abs(spot[1] - target[1]) < RESET_TOLERANCE_PX:
            self._arrive_hits += 1
        else:
            self._arrive_hits = 0

        if self._arrive_hits >= ARRIVE_HITS_NEED:
            self._arrive_hits = 0
            self.running = False
            return True
        # 🔧 超时也算完成 (没激光笔时不卡死)
        if time.ticks_diff(now, self.arrive_start_time) > ARRIVE_TIMEOUT_MS:
            self._arrive_hits = 0
            self.running = False
            return True
        return False

    def _pick_start_idx(self):
        """起点取"离当前光斑最近的路径点", 并进入前置对位阶段

        为什么起点不固定用0号角点: 起跑时光斑通常在A4中心, 离最近的路径点约10cm、
        离最远约18cm —— 取最近的能少飞一截, 对位更快.
        (赛题只要求"沿胶带顺时针一周", 没规定起点, 所以起点可以自由选)
        """
        self._settling = False
        self._start_idx = 0
        self._settle_hits = 0
        if not self.path_points or self.angle_mode:
            return
        red = vision.get_red_coords()
        fx, fy = (red[0], red[1]) if red else (config.CAM_HOME_PX, config.CAM_HOME_PY)
        bd = None
        for i in range(len(self.path_points)):
            d = abs(self.path_points[i][0] - fx) + abs(self.path_points[i][1] - fy)
            if bd is None or d < bd:
                bd, self._start_idx = d, i
        self._settling = True
        print("[TRAJ] start idx={} (laser {}px away)".format(self._start_idx, bd))

    def _step_lap(self, now):
        """边线/胶带一周(基本2/3/4): 路径点按时间推进 + 伺服每帧跑

        为什么不再"到位才走下一点": 每个点最少要凑 SERVO_MED_N(3) 帧中位数 +
        ARRIVE_HITS_NEED(3) 次到位去抖 = 5 个 tick = 500ms, 124 点就是 62 秒,
        30 秒时限下结构上跑不完. 改成"按 elapsed 算进度"之后:
        - 整圈时间 = 点数 × 间隔 ≈ 12.4 秒, 与帧率、与跟随好坏都无关
        - 掉帧只会让进度一次跳多格, 不影响时长
        - 某一段跟不上不会滚雪球, 只是那一段的跟随误差变大
        """
        n = len(self.path_points)

        # ── 前置对位: 先把光斑搬到起点、等它贴上去, **然后才开始计时** ──
        #    不这么做的话, 起跑那一两秒光斑还在从A4中心往路径上飞, 这段路正好压在
        #    **起点那条边**上(2026-09-30现场: "其余三条边正常, 起点那条边在乱扭"),
        #    而且那1~2秒还被算进了整圈时间
        if self._settling:
            px, py = self.path_points[self._start_idx]
            if self.path_idx != self._start_idx:
                self.path_idx = self._start_idx
                if not self.angle_mode:
                    vision.set_path_idx(self._start_idx)
            self._servo_track(px, py)
            self._cur_target = (px, py)
            spot = vision.get_red_coords()
            waited = time.ticks_diff(now, self.start_time)
            # 要连续 LAP_SETTLE_HITS 帧都在容差内: 免得在"扭的过程中正好路过"就开跑
            if spot and abs(spot[0] - px) + abs(spot[1] - py) <= LAP_SETTLE_TOL_PX:
                self._settle_hits += 1
            else:
                self._settle_hits = 0
            if self._settle_hits >= LAP_SETTLE_HITS or waited >= LAP_SETTLE_TIMEOUT_MS:
                self._settling = False
                self.start_time = now      # ← 整圈计时从"已经贴住起点"这一刻起
                self._hold_frames = 0      # 已经贴住了, 不用再压积分
                print("[TRAJ] settled idx={} {}ms -> lap starts".format(self._start_idx, waited))
            return False

        lap_ms = n * config.LAP_INTERVAL_MS
        elapsed = time.ticks_diff(now, self.start_time)

        # 🔧 先判完成再算目标: 否则最后一帧 idx 会绕回 0, 光斑突然去够起点那一点
        if elapsed >= lap_ms:
            self.running = False
            print("[TRAJ] lap done {}ms ({} pts)".format(elapsed, n))
            self._dump_outliers()   # 把"抽那一下"的原始数据落盘(没有离群就不写)
            return True

        # 起点是 _start_idx, 之后按时间从它往前推进: 目标每 LAP_INTERVAL_MS 前进一格
        # (一格 ≈ 2.7px ≈ 0.42° ≈ 0.8cm@1m)
        # 📌 2026-10-01 试过一个改进(**已撤回, 保持现状**): 把进度改成浮点、在相邻路径点之间
        #    线性插值, 让指令每帧只前进~1.5px 的连续微动, 消除"每100ms跳一格"的量化
        #    (F32C 对每个0.42°小步都要"起步-刹停"). 写好后**从未上板验证过**, 就撞上了
        #    另一场故障(追假块/场地变动), 为避免混淆变量先撤回. 需要时能一行加回来
        idx = (self._start_idx + elapsed // config.LAP_INTERVAL_MS) % n
        if idx != self.path_idx:
            self.path_idx = idx
            # 🔧 同步目标点索引给vision, 屏幕蓝色目标点沿边线前进
            if not self.angle_mode:
                vision.set_path_idx(idx)
        tx, ty = self.path_points[idx]
        self._cur_target = (tx, ty)

        if self.angle_mode:
            # 🔧 开环兜底路径存的是"电机角度"不是像素, 必须走这一支.
            #    误当像素喂给 _servo_track 会算出 -41°/+28° 把云台甩出画面, 光斑跑出ROI再也回不来
            self.f32c.set_position(F32C_AXIS_X, tx)
            self.f32c.set_position(F32C_AXIS_Y, ty)
        else:
            self._servo_track(tx, ty)
        return False

    def _servo_track(self, px, py):
        """边线/胶带跟随伺服: 粗对准(前馈) + 积分修正(反馈)
        Args:
            px, py: 当前目标像素(取自开跑瞬间冻结的路径快照)

        与 _move_to_pixel(复位仍在用) 的区别: 修正量**跨目标累积**, 并与粗对准分开存放.
        目标每 100ms 就变一次, 若像 _move_to_pixel 那样每次重置回粗对准, 等于每帧只做一次
        比例修正, 稳态误差收不干净 -> 半开环

        ★结构: 指令角 = 粗对准(当前目标) + 积分修正量
        - 粗对准项跟着目标走, 就是**前馈**: 目标前进多少像素, 这一项就前进多少度
        - 积分修正量只累积"光斑-目标"的视觉残差, 不管目标在哪
        为什么要前馈 —— 2026-09-30 实测踩的坑: 只留反馈时, 一旦光斑连续丢几帧
        (_spot_median 要求连续 SERVO_MED_N 帧都检出), 指令角就**原地冻结**:
        蓝点继续沿边线走, 激光却一动不动, 而且光斑一旦滑出 ROI 就永远回不来(死锁).
        有了前馈, 丢帧时自动退化成纯航位推算(按标定好的 PIX2DEG 沿胶带走), 光斑回来
        就立刻恢复闭环 —— 标定准的话开环这一段也是贴着胶带的.
        前馈在这里是**安全的**: 路径是开跑瞬间的冻结快照, 目标不会因重检测而跳变,
        所以不存在"前馈把指令硬拽一下"的风险
        """
        # 🔧 目标突跳检测(anti-windup): 起步/换路径时目标会跳几十px, 这时候先别积分,
        #    让前馈把云台搬过去, 免得积分冲出一个远超需要的量(现场看到"起步弹一下")
        if self._last_tgt is not None:
            if abs(px - self._last_tgt[0]) + abs(py - self._last_tgt[1]) > TRACK_JUMP_PX:
                self._hold_frames = TRACK_JUMP_HOLD_FRAMES
        self._last_tgt = (px, py)
        if self._hold_frames > 0:
            self._hold_frames -= 1

        # 🔧 lap 用**当帧原始读数**, 不用3帧中位数:
        #    中位数带来约3帧延迟, 加上增益就形成小幅极限环 —— 现象是"激光一直小幅抖动",
        #    而中位数把误差平滑掉了, 所以 err 读数看着只有1~2px(仪表说没事, 眼睛说在抖).
        #    现在假块有大过滤器: 面积闸门(SPOT_MAX_AREA) + 积分死区, 中位数的价值已经不大了.
        spot = vision.get_red_coords()
        # 🔧 跳变门控(见 SPOT_GATE_PX 说明): 与上次**被采纳**的读数差太远 -> 判为坏读数,
        #    这一帧当"没看见"处理(不积分; 前馈照常推, 所以激光不会因此停住)
        if spot:
            if self._last_spot is not None and self._gate_rej < SPOT_GATE_MAX_REJECT and \
                    abs(spot[0] - self._last_spot[0]) + abs(spot[1] - self._last_spot[1]) > SPOT_GATE_PX:
                self._gate_rej += 1
                spot = None
            else:
                self._last_spot = spot
                self._gate_rej = 0
        if spot and self._hold_frames == 0:
            err_x = spot[0] - px   # 光斑相对目标偏差(像素): >0=光斑在目标右侧
            err_y = spot[1] - py   # >0=光斑在目标下方
            # ★不设上限(标定偏差可能几十px, 必须能一路补回来), 但有**死区**:
            #   已经够准的那一轴就别再动, 免得被质心噪声推着随机游走(起步"左右扭")
            if abs(err_x) > TRACK_DEADBAND_PX:
                self._track_corr_x -= _clamp_step(err_x * config.PIX2DEG_X * config.SERVO_GAIN_LAP * config.SPOT_DIR_X)
            if abs(err_y) > TRACK_DEADBAND_PX:
                self._track_corr_y -= _clamp_step(err_y * config.PIX2DEG_Y * config.SERVO_GAIN_LAP * config.SPOT_DIR_Y)
            # 🔧 离群记录(见 SERVO_OUTLIER_PX 说明): 把"抽一下"的原始数据留到文件里
            if abs(err_x) + abs(err_y) > SERVO_OUTLIER_PX:
                self._log_outlier(px, py, spot, err_x, err_y)

        self.f32c.set_position(F32C_AXIS_X,
            config.HOME_ANGLE_X + (px - config.CAM_HOME_PX) * config.PIX2DEG_X * config.SPOT_DIR_X + self._track_corr_x)
        self.f32c.set_position(F32C_AXIS_Y,
            config.HOME_ANGLE_Y + (py - config.CAM_HOME_PY) * config.PIX2DEG_Y * config.SPOT_DIR_Y + self._track_corr_y)

    def _log_outlier(self, px, py, spot, ex, ey):
        """记一行离群样本(光斑离目标太远的那一帧) —— 落盘见 _dump_outliers"""
        if len(self._outliers) >= SERVO_OUTLIER_MAX:
            return
        self._outliers.append((time.ticks_ms(), self.path_idx,
                               int(px), int(py), spot[0], spot[1], int(ex), int(ey),
                               self._track_corr_x, self._track_corr_y, self._hold_frames,
                               vision.get_spot_area()))   # 块面积: 假块通常和真光斑不是一个尺寸

    def _dump_outliers(self):
        """把离群记录追加到 /sdcard/traj.log (跑完一圈时落盘)
        ⚠️ 为什么落盘: main.py 运行时串口看不到实时 print, 屏幕上又是一瞬 ——
           要把"抽那一下"的原始数据留下来事后读. 读法见 progress.md
        """
        if not self._outliers:
            return
        try:
            n = len(self._outliers)
            with open('/sdcard/traj.log', 'a') as f:
                for r in self._outliers:
                    f.write("OUT t{} idx{} tgt{},{} spot{},{} err{},{} a{} corr{:.1f},{:.1f} hold{}"
                            .format(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7],
                                    r[11], r[8], r[9], r[10]))
                    f.write("\n")
            print("[TRAJLOG] {} outliers dumped".format(n))
        except Exception as ex:
            print("[TRAJLOG] fail:", ex)
        self._outliers = []

    def _move_to_pixel(self, px, py, spot):
        """移动激光到目标像素坐标 (固定机位: 粗对准起头 + 视觉伺服积分逼近)
        Args:
            spot: 本轮的光斑读数(滚动中位数), None=没看到光斑
        - 粗对准: 用config标定的HOME映射算目标角度 (光斑不可见时也能动, 不会卡死)
        - 视觉伺服(积分): 目标不变时在"上一次指令角"上继续修正, 而不是每次从粗对准重算.
          这样误差会逐次累积修正到 0, 与电机角度零点/回差无关.
          (旧写法是"粗对准+一次比例修正", 稳态误差 = 粗对准误差×56%, 零点一漂就收不回来)
        """
        # 目标变了 -> 用粗对准重新起头, 并清掉上一次的累积
        if (px, py) != self._servo_target:
            self._servo_target = (px, py)
            self._servo_angle_x = config.HOME_ANGLE_X + (px - config.CAM_HOME_PX) * config.PIX2DEG_X * config.SPOT_DIR_X
            self._servo_angle_y = config.HOME_ANGLE_Y + (py - config.CAM_HOME_PY) * config.PIX2DEG_Y * config.SPOT_DIR_Y

        # 视觉伺服: 按"光斑-目标"像素误差累积修正
        # SPOT_DIR约定: 电机角增大时光斑的移动方向, 伺服与粗对准用同一符号保证方向一致
        if spot:
            err_x = spot[0] - px   # 光斑相对目标偏差(像素): >0=光斑在目标右侧
            err_y = spot[1] - py   # >0=光斑在目标下方
            self._servo_angle_x -= err_x * config.PIX2DEG_X * config.SERVO_GAIN * config.SPOT_DIR_X
            self._servo_angle_y -= err_y * config.PIX2DEG_Y * config.SERVO_GAIN * config.SPOT_DIR_Y

        # 发送位置指令
        self.f32c.set_position(F32C_AXIS_X, self._servo_angle_x)
        self.f32c.set_position(F32C_AXIS_Y, self._servo_angle_y)

    def _spot_median(self):
        """伺服用光斑读数: 最近SERVO_MED_N次采样的中位数
        用中位数不用均值: 偶尔一帧误检(把反光当光斑)不会把结果带偏.
        缓存没满 / 连续丢帧 -> 返回None, 伺服自动退化为纯粗对准, 不会卡死
        """
        red = vision.get_red_coords()
        if not red:
            self._spot_buf = []
            return None
        self._spot_buf.append((red[0], red[1]))
        if len(self._spot_buf) > SERVO_MED_N:
            self._spot_buf.pop(0)
        if len(self._spot_buf) < SERVO_MED_N:
            return None
        xs = sorted(p[0] for p in self._spot_buf)
        ys = sorted(p[1] for p in self._spot_buf)
        m = len(xs) // 2
        return (xs[m], ys[m])

    def get_target(self):
        """当前目标像素(开跑时冻结的路径上的点); 没在跑/开环模式返回 None

        给 main.py 画"伺服误差时间迹线"用: 光斑减它就是**伺服自己的跟踪误差**,
        与矩形检测无关 —— 胶带检不出来(rect:--)时也能看激光到底抖不抖.
        用实时检出的胶带当参照不行: 检出帧间会翻(杂块/保持框), 参照物坏了读数量出来全是垃圾.
        """
        if not self.running or self.angle_mode or not self.path_points:
            return None
        # 🔧 返回**帧间插值后**的目标, 不是整格的路径点:
        #    迹线画的是"光斑 - 目标", 若参照用整格点, 目标本身每100ms跳2.7px,
        #    迹线上就会长出一条 ±1.35px 的锯齿 —— 而整条迹线满格才 ±10px、判分只认 ±3px,
        #    这点锯齿会被当成"激光在抖". 参照物必须和指令用同一个目标
        return self._cur_target

    def get_progress(self):
        """获取进度信息
        Returns:
            dict: {mode, path_idx, total, elapsed_ms, running}
        """
        now = time.ticks_ms()
        return {
            'mode': self.current_mode,
            'path_idx': self.path_idx,
            'total': len(self.path_points),
            'elapsed_ms': time.ticks_diff(now, self.start_time) if self.running else 0,
            'running': self.running,
        }
