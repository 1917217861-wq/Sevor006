"""
极简 UART1 TX 测试: 只做一件事 — 配置 pin9=UART1_TXD, 疯狂发送
目的: 验证 pin9 是否能输出 UART 信号, 排除 main.py 其他逻辑干扰
运行后 量 pin9 对 GND 电压:
  ~3V 上下波动 = K230 TX 正常 → 之前 main.py 跑着但 pin9 仍 0.369V 说明量错引脚
  还是 0.369V = pin9 没被配置成 TX, 或板子硬件问题
"""
from machine import FPIOA, UART
import utime

# pin9 = UART1_TXD, pin10 = UART1_RXD
fpioa = FPIOA()
fpioa.set_function(9, FPIOA.UART1_TXD)
fpioa.set_function(10, FPIOA.UART1_RXD)

uart = UART(1, baudrate=115200)

print("[TEST] UART1 init OK, pin9 should be TX...")
print("[TEST] Now measure pin9 voltage against GND (USB metal shell)")
print("[TEST] Expect ~3V fluctuating, not 0.369V")

# 疯狂发送, 让 pin9 持续跳变
data = b'\x7A\x01\x01\x00\x01\x7D\x7B'  # 使能帧示例
while True:
    uart.write(data)
    utime.sleep_ms(10)
