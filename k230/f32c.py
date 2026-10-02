"""
F32C 协议层 (Python版, 翻译自 STM32 f32c.c)
WHEELTEC F32C 二自由度无刷电机云台
- X轴电机 ID=1 (水平)   ← 官方默认1号
- Y轴电机 ID=2 (俯仰)   ← 官方默认2号

帧格式: 7A + ID + CMD + DATA(0~4B) + BCC + 7B
BCC = 帧头7A ^ ID ^ CMD ^ DATA... (从帧头开始异或, 官方手册验证)
控制流程: 1.使能 → 2.选模式(上电默认速度模式,位置控制必须先发CMD00) → 3.速度 → 4.目标角度
通信: TTL 3.3V UART, 115200 8N1
"""
import time
from config import (F32C_AXIS_X, F32C_AXIS_Y)


# === 帧头/帧尾 ===
FRAME_HEAD = 0x7A
FRAME_TAIL = 0x7B

# === 功能码 ===
CMD_ENABLE  = 0x06   # 使能
CMD_DISABLE = 0x05   # 失能
CMD_MODE    = 0x00   # 模式选择
CMD_SPEED   = 0x01   # 速度设置
CMD_POSITION = 0x02  # 多圈位置(绝对)
CMD_ACCEL   = 0x07   # 加速度
CMD_SAVE    = 0x08   # 保存参数
CMD_CLEAR_MULTI = 0x09   # 多圈角度清零
CMD_FEEDBACK = 0x0E  # 数据反馈请求

# === 模式选择参数 (CMD_MODE 的2字节数据) ===
MODE_SPEED             = 0x0000  # 速度模式
MODE_MULTI_TURN_T      = 0x0001  # 多圈T型轨迹(默认)
MODE_SINGLE_TURN_T     = 0x0002  # 单圈T型轨迹
MODE_MULTI_TURN_DIRECT = 0x0003  # 多圈直通
MODE_SINGLE_TURN_DIRECT = 0x0004  # 单圈直通

# === 反馈类型 ===
FB_SPEED       = 0x00  # 速度, 单位RPM, 16位有符号
FB_TOTAL_ANGLE = 0x01  # 转过总角度, 单位0.1°, 32位有符号
FB_MECH_ANGLE  = 0x02  # 机械角度, 单位0.1°
FB_ACCEL       = 0x03  # 加速度
FB_VOLTAGE     = 0x04  # 母线电压, 单位0.01V

# 指令间最小间隔 (手册要求≥1ms, 留余量2ms)
INTER_CMD_DELAY_S = 0.002


def calc_bcc(data):
    """计算BCC: data 各字节异或
    官方协议: BCC = 从帧头7A开始到DATA末尾所有字节的异或
    验证(官方手册): 7A^01^06=7D(使能X轴), 7A^02^06=7E(使能Y轴)
    Args:
        data: bytes 或 list[int], 含帧头7A, 不含BCC和帧尾7B
    Returns:
        int: BCC校验值 (0~255)
    """
    bcc = 0
    for b in data:
        bcc ^= b
    return bcc & 0xFF


def to_signed32(v):
    """32位无符号转有符号"""
    if v & 0x80000000:
        return v - 0x100000000
    return v


def to_signed16(v):
    """16位无符号转有符号"""
    if v & 0x8000:
        return v - 0x10000
    return v


class F32C:
    """F32C 云台控制类"""

    def __init__(self, uart):
        """初始化
        Args:
            uart: 已配置好的 UART 对象 (K230 CanMV UART)
        """
        self.uart = uart
        # 电机状态快照 (反馈帧更新)
        self.enabled = 0          # 使能位图: bit0=X, bit1=Y
        self.total_angle_x = 0    # X转过总角度, 单位0.1°
        self.total_angle_y = 0    # Y转过总角度, 单位0.1°
        self.speed_x = 0          # X速度, RPM
        self.speed_y = 0          # Y速度, RPM
        self.bus_voltage = 0      # 母线电压, 单位0.01V

    # === 底层发送 ===
    def _send_raw(self, frame):
        """发送原始帧并等待指令间隔"""
        self.uart.write(frame)
        time.sleep(INTER_CMD_DELAY_S)

    def _build_frame(self, axis, cmd, data=b''):
        """构造完整帧: 7A + axis + cmd + data + BCC + 7B
        Args:
            axis: 电机ID (0 or 1)
            cmd: 功能码
            data: 数据字段 (bytes, 长度0~4)
        Returns:
            bytes: 完整帧
        """
        payload = bytes([axis, cmd]) + data
        # 官方协议: BCC从帧头7A开始异或 (手册验证: 7A^01^06=7D)
        bcc = calc_bcc(bytes([FRAME_HEAD]) + payload)
        return bytes([FRAME_HEAD]) + payload + bytes([bcc, FRAME_TAIL])

    # === 基础指令 ===
    def enable(self, axis):
        """使能电机
        Args:
            axis: F32C_AXIS_X(1) 或 F32C_AXIS_Y(2)
        官方帧: 7A 01 06 7D 7B (X轴使能), 7A 02 06 7E 7B (Y轴使能)
        现象: LED从常亮变为快速闪烁
        """
        frame = self._build_frame(axis, CMD_ENABLE)
        self.enabled |= (1 << axis)
        return self._send_raw(frame)

    def disable(self, axis):
        """失能电机
        帧示例: 7A 00 05 05 7B (X轴失能)
        """
        frame = self._build_frame(axis, CMD_DISABLE)
        self.enabled &= ~(1 << axis)
        return self._send_raw(frame)

    def set_mode(self, axis, mode):
        """设置控制模式 (位置控制前必须调用! 上电默认速度模式)
        Args:
            mode: MODE_MULTI_TURN_T 等
        官方帧: 7A 02 00 00 01 79 7B (Y轴设为多圈T)
        """
        data_hi = (mode >> 8) & 0xFF
        data_lo = mode & 0xFF
        frame = self._build_frame(axis, CMD_MODE, bytes([data_hi, data_lo]))
        return self._send_raw(frame)

    def set_speed(self, axis, rpm):
        """设置目标速度
        Args:
            rpm: 速度, RPM, 有符号 (正=正转, 负=反转)
        帧示例: 7A 00 01 00 64 65 7B (X轴100RPM)
                   7A 00 01 FF 9C 64 7B (X轴-100RPM)
        """
        v = rpm & 0xFFFF  # 16位有符号补码
        data_hi = (v >> 8) & 0xFF
        data_lo = v & 0xFF
        frame = self._build_frame(axis, CMD_SPEED, bytes([data_hi, data_lo]))
        return self._send_raw(frame)

    def set_position(self, axis, angle_deg):
        """设置多圈绝对位置 (需先set_mode设为多圈T模式)
        Args:
            angle_deg: 目标角度, 度, 有符号 (正=正方向, 负=反方向)
                       内部 ×10 转 32位补码
        官方帧: 7A 02 02 00 00 0E 10 64 7B (Y轴360°)
        """
        v = int(angle_deg * 10) & 0xFFFFFFFF  # 32位有符号补码
        data = bytes([
            (v >> 24) & 0xFF,
            (v >> 16) & 0xFF,
            (v >> 8) & 0xFF,
            v & 0xFF,
        ])
        frame = self._build_frame(axis, CMD_POSITION, data)
        return self._send_raw(frame)

    def set_accel(self, axis, accel):
        """设置加速度
        Args:
            accel: 加速度, 单位转/s²
        帧示例: 7A 00 07 00 64 63 7B (X轴100转/s²)
        """
        data_hi = (accel >> 8) & 0xFF
        data_lo = accel & 0xFF
        frame = self._build_frame(axis, CMD_ACCEL, bytes([data_hi, data_lo]))
        return self._send_raw(frame)

    def clear_multi_turn(self, axis):
        """多圈角度清零: 把当前物理位置定义为0度 (之后发0度就回到这里)
        官方帧: 7A 01 09 72 7B (X轴), 7A 02 09 71 7B (Y轴)
        用途: F32C多圈零点掉电不保持(实测2026-09-27), 每次上电对中后调一次,
              config 的 HOME_ANGLE 即可恒为 (0,0), 不再依赖零点持久性
        """
        return self._send_raw(self._build_frame(axis, CMD_CLEAR_MULTI))

    def save_params(self, axis):
        """保存参数到驱动器 (是否含零点待实测)
        官方帧: 7A 01 08 73 7B (X轴), 7A 02 08 70 7B (Y轴)
        """
        return self._send_raw(self._build_frame(axis, CMD_SAVE))

    def request_feedback(self, axis, fb_type):
        """请求数据反馈
        Args:
            fb_type: FB_SPEED / FB_TOTAL_ANGLE / FB_MECH_ANGLE / FB_ACCEL / FB_VOLTAGE
        帧示例: 7A 00 0E 01 0F 7B (请求X轴总角度)
        """
        frame = self._build_frame(axis, CMD_FEEDBACK, bytes([fb_type]))
        return self._send_raw(frame)

    # === 反馈帧解析 ===
    def parse_feedback(self, buf):
        """解析反馈帧: 7A + ID + TYPE + DATA(4B) + BCC + 7B (9字节)
        Args:
            buf: bytes, 长度应为9
        Returns:
            bool: True=解析成功, False=解析失败
        """
        if len(buf) != 9:
            return False
        if buf[0] != FRAME_HEAD or buf[8] != FRAME_TAIL:
            return False
        # BCC = 帧头7A ^ ID ^ TYPE ^ DATA(4B), 官方手册反馈帧验证:
        # 7A 02 00 00 00 00 64 1C 7B → 7A^02^00^00^00^00^64 = 1C ✓
        bcc = calc_bcc(bytes([FRAME_HEAD]) + bytes(buf[1:7]))  # 7字节: 帧头+ID+TYPE+DATA
        if bcc != buf[7]:
            return False
        motor_id = buf[1]
        fb_type = buf[2]
        v = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6]
        v = to_signed32(v)

        if motor_id == F32C_AXIS_X:
            if fb_type == FB_SPEED:
                self.speed_x = to_signed16(v & 0xFFFF)
            elif fb_type == FB_TOTAL_ANGLE:
                self.total_angle_x = v
        elif motor_id == F32C_AXIS_Y:
            if fb_type == FB_SPEED:
                self.speed_y = to_signed16(v & 0xFFFF)
            elif fb_type == FB_TOTAL_ANGLE:
                self.total_angle_y = v
        if fb_type == FB_VOLTAGE:
            self.bus_voltage = v & 0xFFFF
        return True

    # === 调试 ===
    def test_frame(self):
        """调试: 发X轴使能帧 7A 00 06 06 7B, 验证UART通路"""
        self.enable(F32C_AXIS_X)

    def test_position_90(self):
        """调试: X轴转90° (用于回校后实测)
        帧: 7A 00 02 00 00 00 03 84 01 7B (BCC=0x00^0x02^0x00^0x00^0x00^0x03^0x84=0x85?需重算)
        正确帧: 7A 00 02 00 00 03 84 <BCC> 7B
          BCC = 0x00 ^ 0x02 ^ 0x00 ^ 0x00 ^ 0x03 ^ 0x84 = 0x85
          完整: 7A 00 02 00 00 03 84 85 7B
        """
        self.set_position(F32C_AXIS_X, 90.0)
