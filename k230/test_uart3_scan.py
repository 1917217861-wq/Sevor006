"""
UART3 TX 全引脚扫描 (配合 USB-TTL 串口助手使用)
================================================
原理: 轮流把 FPIOA 0~63 号引脚设为 UART3_TXD, 每个引脚发送
      3 个字节, 字节值 = 引脚号 (hex)
操作:
  1. USB-TTL 接电脑, 打开串口助手 115200, 十六进制显示
  2. USB-TTL 的 GND 接 K230 的 GND
  3. USB-TTL 的 RX(接收) 接一根杜邦线当探针
  4. 运行本脚本, 用探针逐个碰 12pin 排针的每个引脚
判读: 串口助手收到数据时, 字节值就是该物理孔的 FPIOA 编号!
      例: 收到 20 20 20 → 这个孔是 pin32 (0x20=32)
"""

import utime
from machine import FPIOA, UART

SCAN_ROUND_MS = 3000   # 每轮扫描间隔, 供你换探针位置
PIN_PER_MS = 150       # 每个引脚发送持续时间

fpioa = FPIOA()
uart = UART(UART.UART3, 115200)

print("=== UART3 TX 全引脚扫描开始 ===")
print("用USB-TTL的RX探针逐个碰排针引脚, 串口助手(hex模式)收到")
print("的字节值 = 该孔的FPIOA编号 (例如 20 20 20 = pin32)")
print("每轮扫完歇3秒, 共扫描 0~63 号引脚")

round_num = 0
while True:
    round_num += 1
    print(f"\n--- 第{round_num}轮扫描开始 ---")
    for pin in range(64):
        try:
            fpioa.set_function(pin, FPIOA.UART3_TXD)
        except Exception:
            continue   # 该引脚无UART3_TXD功能, 跳过
        uart.write(bytes([pin]) * 3)
        utime.sleep_ms(PIN_PER_MS)
    print(f"--- 第{round_num}轮结束, 歇{SCAN_ROUND_MS//1000}秒, 换下一个探针位置 ---")
    utime.sleep_ms(SCAN_ROUND_MS)
