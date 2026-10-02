"""
显示 A/B 对照测试: 定位"主循环正常但屏幕无画面"问题
====================================================
Phase A: 直接显示摄像头图 (test_hw.py 的方式, 已知能用)
Phase B: 先画进自建800x480缓冲再显示 (main.py 的方式, 嫌疑对象)

操作: 冷启动板子 -> 连IDE -> 运行本脚本 -> 盯屏幕, 记录A/B哪个阶段有画面
每个阶段8秒, 终端会打印当前阶段
"""
import utime, os
from media.sensor import *
from media.display import *
from media.media import *
import image

PHASE_MS = 8000  # 每阶段时长

sensor = None
try:
    sensor = Sensor(id=2)
    sensor.reset()
    sensor.set_framesize(width=640, height=360, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    Display.init(Display.ST7701, width=800, height=480, to_ide=False)
    MediaManager.init()
    sensor.run()

    # 预览用自建缓冲 (和 main.py 完全相同的创建方式)
    img_buf = image.Image(800, 480, image.RGB565)

    t0 = utime.ticks_ms()
    phase = 0  # 0=A, 1=B
    print(">>> Phase A: 直接显示摄像头图 (8秒), 盯屏幕 <<<")

    while True:
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        elapsed = utime.ticks_diff(utime.ticks_ms(), t0)

        if phase == 0:
            # A: 直接显示传感器图, 居中
            Display.show_image(img, x=(800 - 640) // 2, y=(480 - 360) // 2)
            if elapsed > PHASE_MS:
                phase = 1
                t0 = utime.ticks_ms()
                print(">>> Phase B: 自建缓冲方式显示 (8秒), 盯屏幕 <<<")
        else:
            # B: 画进自建缓冲再显示 (main.py 的渲染路径)
            img_buf.clear(roi=(0, 0, 800, 480))
            img_buf.draw_image(img, 0, 0)
            img_buf.draw_string_advanced(10, 10, 36, "PHASE B", (255, 0, 0))
            Display.show_image(img_buf)
            if elapsed > PHASE_MS:
                print(">>> 测试结束, 两阶段都看完了吗? <<<")
                break

        utime.sleep_ms(30)

except BaseException as e:
    print("[AB] 异常:", e)
finally:
    try:
        if sensor is not None:
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()
    except Exception:
        pass
print("[AB] done")
