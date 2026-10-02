"""
查找 pin33 物理位置 (配合一根杜邦线短接)
================================================
原理: pin32 持续发数据, pin33 负责收。
      你用杜邦线把 pin32 和它旁边的孔逐个短接,
      终端一旦打印收到数据 → 你当前短接的那个孔就是 pin33!
注意:
  - 每次只短接一个孔, 碰一下就松开, 不要长时间短接
  - 如果板子突然重启/屏幕灭, 立刻停止短接 (可能碰到电源脚)
"""

import utime
from machine import FPIOA, UART

fpioa = FPIOA()
fpioa.set_function(32, FPIOA.UART3_TXD)
fpioa.set_function(33, FPIOA.UART3_RXD)
uart = UART(UART.UART3, 115200)

print("=== 查找 pin33 ===")
print("操作: 杜邦线一端插 pin32, 另一端去碰 pin32 旁边的孔")
print("      碰到某个孔时终端打印 [收到] → 那个孔就是 pin33")
print("      每次碰一下就松开, 逐个试旁边的孔\n")

burst = bytes([0x55] * 6)
count = 0
while True:
    count += 1
    uart.write(burst)
    utime.sleep_ms(120)
    buf = b''
    while uart.any():
        buf += uart.read(uart.any())
    if len(buf) >= 6 and buf[:6] == burst:
        print(f"[收到] 第{count}轮: 完整匹配! 你现在短接的孔 = pin33 ✓")
        break
    elif len(buf) > 0:
        print(f"[收到] 第{count}轮: 收到 {len(buf)} 字节: {buf.hex(' ')} (接触不良,再碰一次)")
    if count % 20 == 0:
        print(f"  ...已发送{count}轮, 继续短接尝试...")
    utime.sleep_ms(80)
