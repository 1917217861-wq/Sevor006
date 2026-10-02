"""
UART3 双方向回环测试 (pin32 与 pin33 仍保持短接)
================================================
用途: 自动尝试两种 TX/RX 引脚分配, 判断亚博板的 UART3 方向
操作: pin32-pin33 短接不变, 直接运行
预期: 两种分配中某一种打印 [OK], 则 config.py 需按该方向修改
"""

import utime
from machine import FPIOA, UART

TEST_DATA = bytes([0x7A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x64, 0x1F, 0x7B])


def try_orientation(name, tx_pin, rx_pin):
    print(f"\n=== 尝试分配 {name}: TX=pin{tx_pin}, RX=pin{rx_pin} ===")
    try:
        fpioa = FPIOA()
        fpioa.set_function(tx_pin, FPIOA.UART3_TXD)
        fpioa.set_function(rx_pin, FPIOA.UART3_RXD)
        uart = UART(UART.UART3, 115200)
    except Exception as e:
        print(f"  [XX] 初始化失败: {e}")
        return False

    ok = 0
    for i in range(3):
        uart.write(TEST_DATA)
        utime.sleep_ms(50)
        buf = b''
        start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start) < 200:
            n = uart.any()
            if n:
                buf += uart.read(n)
            if len(buf) >= len(TEST_DATA):
                break
            utime.sleep_ms(5)
        if buf == TEST_DATA:
            ok += 1
        utime.sleep_ms(50)

    try:
        uart.deinit()
    except Exception:
        pass

    if ok >= 2:
        print(f"  [OK] {ok}/3 次成功 → 正确方向就是 {name}!")
        return True
    else:
        print(f"  [XX] 仅 {ok}/3 次成功")
        return False


print("=== UART3 双方向回环测试 (pin32-pin33 保持短接) ===")
print("提前提醒: 板子启动会自动跑main.py, 先点IDE停止键再运行本脚本")

a_ok = try_orientation("A: 原配置", 32, 33)
b_ok = try_orientation("B: 反向", 33, 32)

print("\n" + "=" * 50)
if b_ok and not a_ok:
    print(">>> 结论: 亚博板方向与config相反!")
    print(">>> 修改: config.py 里 F32C_TX_PIN=33, F32C_RX_PIN=32")
elif a_ok and not b_ok:
    print(">>> 结论: 原配置方向正确, 问题在短接线/物理引脚位置上")
elif a_ok and b_ok:
    print(">>> 两种都通? 短接线可能一直没接好, 重新短接再跑一次")
else:
    print(">>> 两种都失败: pin32/33 物理位置可能不对 (丝印编号与FPIOA编号不一致)")
    print(">>> 下一步: 用USB-TTL接pin32, 串口助手看K230有没有发数据")
