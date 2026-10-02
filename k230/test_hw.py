"""
亚博 K230 豪华版 - 到货后最简硬件测试脚本
==========================================
用途: 逐模块验证硬件, 确认 sensor_id / display / UART 引脚

运行方式: CanMV IDE 连接 K230, 打开本脚本, 点运行按钮
预期结果: 串行终端打印每项测试的 [OK]/[XX]

测试项:
  1. Sensor ID 扫描 + 屏幕显示 (试 2/1/0, 找到能出图的 id, 画面显示3秒)
  2. Touch 触摸检测
  3. UART1 自发自收 (pin9 TX / pin10 RX, 需跳线短接)
  4. UART3 自发自收 (pin32 TX / pin33 RX, 需跳线短接)
"""

import utime
from machine import FPIOA, UART

# ============================================================
# 测试1: Sensor ID 扫描 + Display 显示
# 完整初始化链: sensor.reset -> Display.init -> MediaManager.init
#               -> sensor.run -> snapshot
# ============================================================
def test_sensor_display():
    print("\n=== 测试1: Sensor ID 扫描 + Display ===")
    found_id = None

    for sid in [2, 1, 0]:
        sensor = None
        try:
            from media.sensor import Sensor, CAM_CHN_ID_0
            from media.display import Display
            from media.media import MediaManager

            print(f"  尝试 Sensor id={sid} ...")
            sensor = Sensor(id=sid)
            sensor.reset()
            sensor.set_framesize(width=640, height=360, chn=CAM_CHN_ID_0)
            sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

            # 屏幕显示 (亚博豪华版, 先按 ST7701 试)
            Display.init(Display.ST7701, width=800, height=480, to_ide=False)
            MediaManager.init()
            sensor.run()

            # 采3帧让 AWB/曝光稳定, 取最后一帧
            img = None
            for _ in range(3):
                img = sensor.snapshot(chn=CAM_CHN_ID_0)
                utime.sleep_ms(100)

            if img is not None and img.width() > 0:
                print(f"  [OK] Sensor id={sid} 出图成功: {img.width()}x{img.height()}")
                found_id = sid
                # 画面显示到屏幕, 持续3秒供肉眼确认
                Display.show_image(img, x=(800 - img.width()) // 2,
                                        y=(480 - img.height()) // 2)
                print(f"  >>> 请看屏幕, 应显示摄像头画面 (3秒后关闭) <<<")
                utime.sleep_ms(3000)
            else:
                print(f"  [XX] Sensor id={sid} snapshot 无图像")

        except Exception as e:
            print(f"  [XX] Sensor id={sid} 失败: {e}")

        # 无论成败, 清理本次尝试的媒体资源, 给下一个 id 干净环境
        try:
            if sensor is not None:
                sensor.stop()
            from media.display import Display
            from media.media import MediaManager
            Display.deinit()
            MediaManager.deinit()
        except Exception:
            pass
        utime.sleep_ms(200)

        if found_id is not None:
            break

    if found_id is None:
        print("  [XX] 所有 Sensor id 都失败! 检查摄像头排线是否插好")
    else:
        print(f"  >>> 结论: 把 config.py 的 SENSOR_ID 改为 {found_id} <<<")
    return found_id

# ============================================================
# 测试2: Touch 触摸
# ============================================================
def test_touch():
    print("\n=== 测试2: Touch 触摸 ===")
    try:
        from machine import TOUCH
    except ImportError:
        print("  [XX] 固件无 TOUCH 模块 (触摸功能不可用, 不影响主流程)")
        return False

    touched = False
    for tp_id in [0, 1]:
        try:
            tp = TOUCH(tp_id)
            print(f"  TOUCH({tp_id}) 初始化成功, 请触摸屏幕 (5秒)...")
            start = utime.ticks_ms()
            while utime.ticks_diff(utime.ticks_ms(), start) < 5000:
                p = tp.read()
                if p:
                    print(f"  [OK] 触摸成功: x={p[0].x}, y={p[0].y}")
                    touched = True
                    break
                utime.sleep_ms(50)
            if touched:
                break
        except Exception as e:
            print(f"  [XX] TOUCH({tp_id}): {e}")
    if not touched:
        print("  未检测到触摸 (不影响电机控制主流程, 可后补)")
    return touched

# ============================================================
# 测试3/4: UART 自发自收
# 前提: 用杜邦线把该路 UART 的 TX 和 RX 引脚短接
# ============================================================
def test_uart(uart_id, tx_pin, rx_pin, name):
    print(f"\n=== 测试 UART{uart_id} ({name}) ===")
    print(f"  请确认已用杜邦线短接: pin{tx_pin}(TX) <-> pin{rx_pin}(RX)")
    print(f"  5秒内自动发送0x55并等待收回...")
    try:
        fpioa = FPIOA()
        fpioa.set_function(tx_pin, getattr(FPIOA, f"UART{uart_id}_TXD"))
        fpioa.set_function(rx_pin, getattr(FPIOA, f"UART{uart_id}_RXD"))
        uart = UART(uart_id, baudrate=115200)
        print(f"  [OK] UART{uart_id} 初始化成功 (TX=pin{tx_pin}, RX=pin{rx_pin})")

        ok = False
        start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start) < 5000:
            uart.write(b'\x55')
            utime.sleep_ms(50)
            if uart.any():
                data = uart.read(1)
                if data and data[0] == 0x55:
                    print(f"  [OK] 自发自收成功 (收到0x55)")
                    ok = True
                    break
        if not ok:
            print(f"  [XX] 未收回数据 - 若引脚已短接则记录结果, 联调时再复核")
        uart.deinit()
        return ok
    except Exception as e:
        print(f"  [XX] UART{uart_id}: {e}")
        return False

# ============================================================
# 主函数
# ============================================================
def main():
    print("=" * 50)
    print("亚博 K230 豪华版 - 硬件测试")
    print("=" * 50)

    sid = test_sensor_display()
    test_touch()
    # UART1 (pin9/10) 对接 STM32; UART3 (pin32/33) 对接 F32C
    test_uart(1, 9, 10, "K230 <-> STM32")
    test_uart(3, 32, 33, "K230 -> F32C")

    print("\n" + "=" * 50)
    print("测试完成! 结果汇总:")
    print(f"  SENSOR_ID 应改为: {sid if sid is not None else '无可用id, 需排查'}")
    print("  把结果发给 AI, 更新 config.py 后进入联调")
    print("=" * 50)

if __name__ == "__main__":
    main()
