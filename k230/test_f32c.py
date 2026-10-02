"""
K230 → F32C 直连测试脚本 (正式链路验证)
==========================================
用途: 用 K230 的 UART3 (pin32/33) 直接控制 F32C, 验证正式控制链路
接线: K230 pin32(TX) → F32C RXD
      K230 pin33(RX) → F32C TXD
      K230 GND       → F32C GND (与电池负极共地, 必须!)

运行: CanMV IDE 连接后点运行
预期: 串行终端打印每步 [OK]/[XX], 电机按指令动作

测试流程:
  1. X轴使能 (观察LED常亮→快闪)
  2. Y轴使能 (观察LED常亮→快闪)
  3. 两轴设为多圈T型轨迹模式
  4. 读X轴总角度反馈 (验证反馈通路)
  5. X轴 0°→90°→0° (观察转动)
  6. Y轴 0°→30°→0° (小角度, K230装在Y轴上会跟着动, 注意留空间!)
"""

import utime
from machine import FPIOA, UART
import config
from f32c import F32C, F32C_AXIS_X, F32C_AXIS_Y, MODE_MULTI_TURN_T, FB_TOTAL_ANGLE, FB_SPEED

# ============================================================
# 反馈读取: 从UART3读原始字节, 找9字节反馈帧交给 f32c.parse_feedback
# ============================================================
def read_feedback(f32c, timeout_ms=300):
    """等待并解析一帧反馈
    Returns:
        (motor_id, fb_type, value) 或 None
    """
    start = utime.ticks_ms()
    buf = bytearray()
    while utime.ticks_diff(utime.ticks_ms(), start) < timeout_ms:
        n = f32c.uart.any()
        if n:
            buf.extend(f32c.uart.read(n))
            # 用游标i在缓冲里找 7A...7B 的9字节帧 (MicroPython bytearray 无pop)
            i = 0
            while len(buf) - i >= 9:
                if buf[i] != 0x7A:
                    i += 1  # 跳过非帧头字节, 对齐帧头
                    continue
                frame = bytes(buf[i:i + 9])
                if frame[8] == 0x7B and f32c.parse_feedback(frame):
                    motor_id, fb_type = frame[1], frame[2]
                    v = (frame[3] << 24) | (frame[4] << 16) | (frame[5] << 8) | frame[6]
                    if v & 0x80000000:  # 32位补码转有符号 (内联, 不依赖卡上f32c.py版本)
                        v -= 0x100000000
                    return (motor_id, fb_type, v)
                i += 1  # 帧尾不对或校验失败, 滑动继续找
            if i:
                buf = buf[i:]  # 丢弃已扫描部分 (bytearray切片, 兼容所有固件)
        utime.sleep_ms(5)
    return None


def ask_feedback(f32c, axis, fb_type, label=""):
    """请求反馈并打印结果"""
    f32c.request_feedback(axis, fb_type)
    r = read_feedback(f32c)
    if r is None:
        print(f"  [XX] {label} 无反馈 (检查TXD/RXD是否接反、共地)")
    else:
        mid, ftype, v = r
        if ftype == FB_TOTAL_ANGLE:
            print(f"  [OK] {label}: 电机ID={mid} 总角度={v/10:.1f}°")
        elif ftype == FB_SPEED:
            print(f"  [OK] {label}: 电机ID={mid} 速度={v}RPM")
        else:
            print(f"  [OK] {label}: 电机ID={mid} 类型={ftype} 原始值={v}")
    return r


def move_and_wait(f32c, axis, angle_deg, max_wait_s=10):
    """发位置指令并轮询反馈等到位 (角度变化<2°视为到位)"""
    print(f"  >>> 电机ID={axis} 目标角度 {angle_deg}° ...")
    f32c.set_position(axis, angle_deg)
    start = utime.ticks_ms()
    last_angle = None
    while utime.ticks_diff(utime.ticks_ms(), start) < max_wait_s * 1000:
        f32c.request_feedback(axis, FB_TOTAL_ANGLE)
        r = read_feedback(f32c, timeout_ms=200)
        if r:
            cur = r[2] / 10.0
            if last_angle is not None and abs(cur - angle_deg) < 2.0:
                print(f"  [OK] 到位: 当前 {cur:.1f}°")
                return True
            last_angle = cur
        utime.sleep_ms(100)
    print(f"  [XX] {max_wait_s}秒内未到位 (最后角度 {last_angle}°), 继续下一步")
    return False


# ============================================================
# 主流程
# ============================================================
def main():
    print("=" * 50)
    print("K230 -> F32C 直连测试")
    print("=" * 50)

    # UART3 初始化 (与 main.py 一致)
    fpioa = FPIOA()
    fpioa.set_function(config.F32C_UART_TX_PIN, FPIOA.UART3_TXD)   # pin32
    fpioa.set_function(config.F32C_UART_RX_PIN, FPIOA.UART3_RXD)   # pin33
    uart = UART(UART.UART3, config.F32C_BAUDRATE)
    f32c = F32C(uart)

    # --- 测试1: 使能 ---
    print("\n=== 测试1: 使能 X轴 (ID=1) ===")
    print("  请观察X轴电机LED: 应从常亮变为快闪")
    f32c.enable(F32C_AXIS_X)
    utime.sleep_ms(500)

    print("\n=== 测试2: 使能 Y轴 (ID=2) ===")
    print("  请观察Y轴电机LED: 应从常亮变为快闪")
    f32c.enable(F32C_AXIS_Y)
    utime.sleep_ms(500)

    # --- 测试3: 模式 + 速度 ---
    print("\n=== 测试3: 两轴设为多圈T型轨迹模式, 巡航速度60RPM ===")
    f32c.set_mode(F32C_AXIS_X, MODE_MULTI_TURN_T)
    f32c.set_mode(F32C_AXIS_Y, MODE_MULTI_TURN_T)
    f32c.set_speed(F32C_AXIS_X, 60)   # T型轨迹巡航速度, 不发的话用寄存器残留值(可能为0/极小)
    f32c.set_speed(F32C_AXIS_Y, 60)
    print("  已发送 (F32C无反馈帧, 无报错即通路正常)")

    # --- 测试4: 反馈 ---
    print("\n=== 测试4: 读反馈 (验证TXD→RXD回传通路) ===")
    ask_feedback(f32c, F32C_AXIS_X, FB_TOTAL_ANGLE, "X轴总角度")
    ask_feedback(f32c, F32C_AXIS_Y, FB_TOTAL_ANGLE, "Y轴总角度")

    # --- 测试5: X轴转动 ---
    print("\n=== 测试5: X轴 0°→90°→0° ===")
    move_and_wait(f32c, F32C_AXIS_X, 90.0)
    utime.sleep_ms(500)
    move_and_wait(f32c, F32C_AXIS_X, 0.0)

    # --- 测试6: Y轴小角度 (K230在Y轴上, 会跟着动!) ---
    print("\n=== 测试6: Y轴 0°→30°→0° (K230会跟着转, 注意留空间!) ===")
    move_and_wait(f32c, F32C_AXIS_Y, 30.0)
    utime.sleep_ms(500)
    move_and_wait(f32c, F32C_AXIS_Y, 0.0)

    print("\n" + "=" * 50)
    print("测试完成! 汇总:")
    print("  - LED快闪了 => 使能通路OK")
    print("  - 反馈有值  => TX/RX交叉接线正确")
    print("  - 电机转动  => 位置控制链路OK")
    print("  全通过 => 正式链路就绪, 可跑 main.py 三路联调")
    print("=" * 50)


main()
