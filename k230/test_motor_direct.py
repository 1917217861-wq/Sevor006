"""
极简电机测试: 直接让 X 轴转 90° 再回来
绕过 trajectory.py, 只测 F32C set_position 是否真的在动
运行后观察 X 轴: 先转到 90°, 停 2 秒, 再转回 0°
"""
import time
from machine import FPIOA, UART
from f32c import F32C, F32C_AXIS_X, F32C_AXIS_Y, MODE_MULTI_TURN_T

fpioa = FPIOA()
fpioa.set_function(9, FPIOA.UART1_TXD)
fpioa.set_function(10, FPIOA.UART1_RXD)

uart = UART(1, baudrate=115200)
f = F32C(uart)

print("[TEST] Step 1: Enable X+Y")
f.enable(F32C_AXIS_X)
f.enable(F32C_AXIS_Y)
time.sleep(0.1)

print("[TEST] Step 2: Set mode to MULTI_TURN_T")
f.set_mode(F32C_AXIS_X, MODE_MULTI_TURN_T)
f.set_mode(F32C_AXIS_Y, MODE_MULTI_TURN_T)
time.sleep(0.1)

print("[TEST] Step 3: Set speed 60 RPM")
f.set_speed(F32C_AXIS_X, 60)
f.set_speed(F32C_AXIS_Y, 60)
time.sleep(0.1)

print("[TEST] Step 4: X axis → 90°")
f.set_position(F32C_AXIS_X, 90.0)
time.sleep(3)  # 等电机转完

print("[TEST] Step 5: X axis → 0°")
f.set_position(F32C_AXIS_X, 0.0)
time.sleep(3)

print("[TEST] Done! Did X axis move?")
