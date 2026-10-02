"""
K230端协议单元测试 - 用官方手册示例帧验证帧构造正确性
不依赖硬件, 可在 PC 上直接运行 (需 Python3)

官方参考帧 (F32C无刷电机使用手册-2026-07-17):
  使能   ID=2: 7A 02 06 7E 7B     BCC = 7A^02^06 = 7E
  使能   ID=1: 7A 01 06 7D 7B     BCC = 7A^01^06 = 7D
  多圈T  ID=2: 7A 02 00 00 01 79 7B
  100RPM ID=2: 7A 02 01 00 64 1D 7B
  360°   ID=2: 7A 02 02 00 00 0E 10 64 7B
  反馈   ID=2: 7A 02 01 00 00 0E 10 67 7B (总角度360°)

运行: python test_protocol.py
"""
import sys
import os

# 把当前目录加入 sys.path 以便 import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from f32c import (
    calc_bcc, to_signed32, to_signed16,
    F32C, FRAME_HEAD, FRAME_TAIL,
    CMD_ENABLE, CMD_DISABLE, CMD_MODE, CMD_SPEED, CMD_POSITION, CMD_ACCEL, CMD_FEEDBACK,
    MODE_MULTI_TURN_T, FB_TOTAL_ANGLE,
)
from stm32_comm import STM32Comm, calc_bcc as stm32_calc_bcc
from config import F32C_AXIS_X, F32C_AXIS_Y, STM32_CMD_MODE, STM32_CMD_PAUSE, STM32_CMD_LASER


class FakeUART:
    """假 UART, 记录所有写入的字节, 模拟 K230 CanMV UART 接口
    同时支持模拟接收: 外部代码写入 rx_buf, poll() 能读到"""
    def __init__(self):
        self.tx_buf = bytearray()  # 发送缓冲(记录发出的字节)
        self.rx_buf = bytearray()  # 接收缓冲(模拟对端发来的字节)

    def write(self, data):
        self.tx_buf.extend(data)

    def any(self):
        return len(self.rx_buf)

    def read(self, n=0):
        if n == 0:
            n = len(self.rx_buf)
        if n <= 0:
            return b''
        data = bytes(self.rx_buf[:n])
        del self.rx_buf[:n]
        return data


def hex_str(b):
    """bytes 转十六进制字符串, 便于对比"""
    return ' '.join(f'{x:02X}' for x in b)


def test_f32c_bcc():
    """测试 F32C BCC 计算 (官方协议: BCC含帧头7A, 从7A异或到DATA末尾)"""
    print("=== 测试 F32C BCC 计算 (对照官方手册示例帧) ===")

    # 官方: 使能ID=2 → 7A 02 06 7E 7B, BCC = 7A^02^06 = 7E
    bcc = calc_bcc([0x7A, 0x02, 0x06])
    assert bcc == 0x7E, f"使能ID=2: 期望7E, 实际{bcc:02X}"
    print(f"  [OK] 使能ID=2: 7A^02^06 = {bcc:02X} (官方7E)")

    # 官方: 使能ID=1 → 7A 01 06 7D 7B, BCC = 7A^01^06 = 7D
    bcc = calc_bcc([0x7A, 0x01, 0x06])
    assert bcc == 0x7D, f"使能ID=1: 期望7D, 实际{bcc:02X}"
    print(f"  [OK] 使能ID=1: 7A^01^06 = {bcc:02X} (官方7D)")

    # 官方: 100RPM ID=2 → 7A 02 01 00 64 1D 7B, BCC = 7A^02^01^00^64 = 1D
    bcc = calc_bcc([0x7A, 0x02, 0x01, 0x00, 0x64])
    assert bcc == 0x1D, f"100RPM: 期望1D, 实际{bcc:02X}"
    print(f"  [OK] 100RPM ID=2: 7A^02^01^00^64 = {bcc:02X} (官方1D)")

    # 官方: 360° ID=2 → 7A 02 02 00 00 0E 10 64 7B, BCC = 7A^02^02^00^00^0E^10 = 64
    bcc = calc_bcc([0x7A, 0x02, 0x02, 0x00, 0x00, 0x0E, 0x10])
    assert bcc == 0x64, f"360°: 期望64, 实际{bcc:02X}"
    print(f"  [OK] 360° ID=2: = {bcc:02X} (官方64)")

    # 官方: 反馈帧 7A 02 01 00 00 0E 10 67 7B, BCC = 67
    bcc = calc_bcc([0x7A, 0x02, 0x01, 0x00, 0x00, 0x0E, 0x10])
    assert bcc == 0x67, f"反馈帧: 期望67, 实际{bcc:02X}"
    print(f"  [OK] 反馈帧 ID=2: = {bcc:02X} (官方67)")
    print()


def test_f32c_frames():
    """测试 F32C 帧构造 (对照官方手册示例帧)"""
    print("=== 测试 F32C 帧构造 ===")
    uart = FakeUART()
    f32c = F32C(uart)

    # 1. 使能 X轴 (ID=1): 官方帧 7A 01 06 7D 7B
    uart.tx_buf.clear()
    f32c.enable(F32C_AXIS_X)
    expected = bytes([0x7A, 0x01, 0x06, 0x7D, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"X轴使能: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] X轴使能: {hex_str(uart.tx_buf)}")

    # 2. 使能 Y轴 (ID=2): 官方帧 7A 02 06 7E 7B
    uart.tx_buf.clear()
    f32c.enable(F32C_AXIS_Y)
    expected = bytes([0x7A, 0x02, 0x06, 0x7E, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"Y轴使能: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] Y轴使能: {hex_str(uart.tx_buf)}")

    # 3. 失能 Y轴 (ID=2): 官方帧 7A 02 05 7D 7B
    uart.tx_buf.clear()
    f32c.disable(F32C_AXIS_Y)
    expected = bytes([0x7A, 0x02, 0x05, 0x7D, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"Y轴失能: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] Y轴失能: {hex_str(uart.tx_buf)}")

    # 4. X轴转90° (多圈位置): data = 900 = 0x0384
    #    BCC = 7A^01^02^00^00^03^84 = FE
    uart.tx_buf.clear()
    f32c.set_position(F32C_AXIS_X, 90.0)
    expected = bytes([0x7A, 0x01, 0x02, 0x00, 0x00, 0x03, 0x84, 0xFE, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"X轴90°: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] X轴转90°: {hex_str(uart.tx_buf)}")

    # 5. X轴转360°: 官方ID=1帧 7A 01 02 00 00 0E 10 67 7B (云台手册)
    uart.tx_buf.clear()
    f32c.set_position(F32C_AXIS_X, 360.0)
    expected = bytes([0x7A, 0x01, 0x02, 0x00, 0x00, 0x0E, 0x10, 0x67, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"X轴360°: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] X轴转360°: {hex_str(uart.tx_buf)} (官方帧一致)")

    # 6. X轴转-360°: data = -3600 = 0xFFFFF1F0
    #    BCC = 7A^01^02^FF^FF^F1^F0 = 78
    uart.tx_buf.clear()
    f32c.set_position(F32C_AXIS_X, -360.0)
    expected = bytes([0x7A, 0x01, 0x02, 0xFF, 0xFF, 0xF1, 0xF0, 0x78, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"X轴-360°: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] X轴转-360°: {hex_str(uart.tx_buf)}")

    # 7. X轴速度100RPM: 官方ID=1帧 7A 01 01 00 64 1E 7B (云台手册)
    uart.tx_buf.clear()
    f32c.set_speed(F32C_AXIS_X, 100)
    expected = bytes([0x7A, 0x01, 0x01, 0x00, 0x64, 0x1E, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"X轴100RPM: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] X轴速度100RPM: {hex_str(uart.tx_buf)} (官方帧一致)")

    # 8. X轴速度-100RPM: 官方ID=2帧 7A 02 01 FF 9C 1A 7B
    #    ID=1: BCC = 7A^01^01^FF^9C = 19
    uart.tx_buf.clear()
    f32c.set_speed(F32C_AXIS_X, -100)
    expected = bytes([0x7A, 0x01, 0x01, 0xFF, 0x9C, 0x19, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"X轴-100RPM: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] X轴速度-100RPM: {hex_str(uart.tx_buf)}")

    # 9. Y轴速度-100RPM: 官方验证 BCC = 7A^02^01^FF^9C = 1A
    uart.tx_buf.clear()
    f32c.set_speed(F32C_AXIS_Y, -100)
    expected = bytes([0x7A, 0x02, 0x01, 0xFF, 0x9C, 0x1A, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"Y轴-100RPM: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] Y轴速度-100RPM: {hex_str(uart.tx_buf)} (官方帧一致)")

    # 10. X轴设多圈T模式: 官方ID=1帧 7A 01 00 00 01 7A 7B (云台手册)
    uart.tx_buf.clear()
    f32c.set_mode(F32C_AXIS_X, MODE_MULTI_TURN_T)
    expected = bytes([0x7A, 0x01, 0x00, 0x00, 0x01, 0x7A, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"X轴多圈T: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] X轴多圈T模式: {hex_str(uart.tx_buf)} (官方帧一致)")

    # 11. 请求X轴总角度反馈: BCC = 7A^01^0E^01 = 74
    uart.tx_buf.clear()
    f32c.request_feedback(F32C_AXIS_X, FB_TOTAL_ANGLE)
    expected = bytes([0x7A, 0x01, 0x0E, 0x01, 0x74, 0x7B])
    assert bytes(uart.tx_buf) == expected, \
        f"请求X轴总角度: 期望 {hex_str(expected)}, 实际 {hex_str(uart.tx_buf)}"
    print(f"  [OK] 请求X轴总角度: {hex_str(uart.tx_buf)}")
    print()


def test_stm32_frames():
    """测试 STM32 指令解析 (对照 STM32 k230_comm.c)"""
    print("=== 测试 STM32 指令解析 ===")
    uart = FakeUART()
    comm = STM32Comm(uart)

    received = []
    comm.on_mode_change = lambda m: received.append(('mode', m))
    comm.on_pause = lambda p: received.append(('pause', p))
    comm.on_laser = lambda o: received.append(('laser', o))

    # 1. 模式 RESET: BB 01 01 00 55 (BCC = 0x01 ^ 0x01 = 0x00)
    received.clear()
    uart.tx_buf.clear()
    # 模拟 STM32 发来的帧, 注入到 UART 接收缓冲区
    uart.rx_buf.extend(bytes([0xBB, 0x01, 0x01, 0x00, 0x55]))
    comm.poll()
    assert received == [('mode', 1)], f"模式RESET: 实际 {received}"
    print(f"  [OK] 模式RESET: BB 01 01 00 55 -> mode=1")

    # 2. 模式 EDGE: BB 01 02 03 55 (BCC = 0x01 ^ 0x02 = 0x03)
    received.clear()
    uart.rx_buf.extend(bytes([0xBB, 0x01, 0x02, 0x03, 0x55]))
    comm.poll()
    assert received == [('mode', 2)], f"模式EDGE: 实际 {received}"
    print(f"  [OK] 模式EDGE: BB 01 02 03 55 -> mode=2")

    # 3. 暂停: BB 02 01 03 55 (BCC = 0x02 ^ 0x01 = 0x03)
    received.clear()
    uart.rx_buf.extend(bytes([0xBB, 0x02, 0x01, 0x03, 0x55]))
    comm.poll()
    assert received == [('pause', 1)], f"暂停: 实际 {received}"
    print(f"  [OK] 暂停: BB 02 01 03 55 -> pause=1")

    # 4. 恢复: BB 02 00 02 55 (BCC = 0x02 ^ 0x00 = 0x02)
    received.clear()
    uart.rx_buf.extend(bytes([0xBB, 0x02, 0x00, 0x02, 0x55]))
    comm.poll()
    assert received == [('pause', 0)], f"恢复: 实际 {received}"
    print(f"  [OK] 恢复: BB 02 00 02 55 -> pause=0")

    # 5. 激光开: BB 03 01 02 55 (BCC = 0x03 ^ 0x01 = 0x02)
    received.clear()
    uart.rx_buf.extend(bytes([0xBB, 0x03, 0x01, 0x02, 0x55]))
    comm.poll()
    assert received == [('laser', 1)], f"激光开: 实际 {received}"
    print(f"  [OK] 激光开: BB 03 01 02 55 -> laser=1")

    # 6. BCC错误帧应被丢弃
    received.clear()
    uart.rx_buf.extend(bytes([0xBB, 0x01, 0x02, 0xFF, 0x55]))  # BCC错
    comm.poll()
    assert received == [], f"BCC错误应丢弃: 实际 {received}"
    print(f"  [OK] BCC错误帧被丢弃")

    # 7. 多帧连续解析 (去重逻辑: 只在状态变化时触发)
    received.clear()
    comm.current_mode = 0      # 重置状态, 确保mode=1被视为变化
    comm.pause_flag = 0
    comm.laser_state = 0
    uart.rx_buf.extend(bytes([
        0xBB, 0x01, 0x01, 0x00, 0x55,  # mode RESET (1→变化)
        0xBB, 0x03, 0x01, 0x02, 0x55,  # laser on (0→变化)
        0xBB, 0x02, 0x01, 0x03, 0x55,  # pause (0→变化)
        0xBB, 0x01, 0x01, 0x00, 0x55,  # mode RESET重复 (1→不变, 应被去重)
    ]))
    comm.poll()
    assert len(received) == 3, f"多帧: 实际 {received}"
    print(f"  [OK] 多帧连续解析: {received}")
    print()


def test_feedback_parse():
    """测试 F32C 反馈帧解析 (对照官方手册反馈示例帧)"""
    print("=== 测试 F32C 反馈帧解析 ===")
    uart = FakeUART()
    f32c = F32C(uart)

    # 1. 官方手册反馈示例: 7A 02 01 00 00 0E 10 67 7B (总角度360°)
    buf = bytes([0x7A, 0x02, 0x01, 0x00, 0x00, 0x0E, 0x10, 0x67, 0x7B])
    ok = f32c.parse_feedback(buf)
    assert ok, "官方反馈帧解析失败"
    assert f32c.total_angle_y == 3600, f"Y轴总角度: {f32c.total_angle_y} (期望3600=360.0°)"
    print(f"  [OK] 官方反馈帧(360°): {hex_str(buf)} -> total_angle_y={f32c.total_angle_y}")

    # 2. 官方速度反馈示例: 7A 02 00 00 00 00 64 1C 7B (速度100RPM)
    buf = bytes([0x7A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x64, 0x1C, 0x7B])
    ok = f32c.parse_feedback(buf)
    assert ok, "官方速度反馈帧解析失败"
    assert f32c.speed_y == 100, f"Y轴速度: {f32c.speed_y} (期望100RPM)"
    print(f"  [OK] 官方速度反馈(100RPM): {hex_str(buf)} -> speed_y={f32c.speed_y}")

    # 3. X轴正角度9000 (即90.0°)
    #    BCC = 7A^01^01^00^00^23^28 = 71
    buf = bytes([0x7A, 0x01, 0x01, 0x00, 0x00, 0x23, 0x28, 0x71, 0x7B])
    ok = f32c.parse_feedback(buf)
    assert ok, "反馈帧解析失败"
    assert f32c.total_angle_x == 9000, f"X轴总角度: {f32c.total_angle_x}"
    print(f"  [OK] X轴总角度9000(即90.0°): {hex_str(buf)} -> total_angle_x={f32c.total_angle_x}")

    # 4. X轴负角度-9000 (即-90.0°)
    #    -9000 = 0xFFFFDCD8, BCC = 7A^01^01^FF^FF^DC^D8 = 7E
    buf = bytes([0x7A, 0x01, 0x01, 0xFF, 0xFF, 0xDC, 0xD8, 0x7E, 0x7B])
    ok = f32c.parse_feedback(buf)
    assert ok, "负角度反馈帧解析失败"
    assert f32c.total_angle_x == -9000, f"X轴负角度: {f32c.total_angle_x}"
    print(f"  [OK] X轴总角度-9000(即-90.0°): {hex_str(buf)} -> total_angle_x={f32c.total_angle_x}")
    print()


def main():
    print("K230端协议单元测试 (官方手册帧校验)")
    print("=" * 50)
    try:
        test_f32c_bcc()
        test_f32c_frames()
        test_stm32_frames()
        test_feedback_parse()
        print("=" * 50)
        print("[PASS] 所有测试通过! 帧构造与官方手册完全一致")
        print()
        print("回校后实测关键帧 (USB-TTL直发, 勾选十六进制发送):")
        print("  ① X轴使能:    7A 01 06 7D 7B   (LED变快闪)")
        print("  ② X轴多圈T:   7A 01 00 00 01 7A 7B")
        print("  ③ X轴转90°:   7A 01 02 00 00 03 84 FE 7B")
        print("  ④ Y轴使能:    7A 02 06 7E 7B")
        print("  ⑤ Y轴多圈T:   7A 02 00 00 01 79 7B")
        print("  ⑥ Y轴转90°:   7A 02 02 00 00 03 84 FD 7B")
        return 0
    except AssertionError as e:
        print(f"[FAIL] 测试失败: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
