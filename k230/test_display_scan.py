"""
LCD驱动扫描v2: 只测固件里真正的3个面板驱动, ST7701额外测两种几何
====================================================================
v1教训: 画布宽度300不是8的倍数, 固件要求图像宽度必须是8的倍数, 全部阶段秒挂
本版画布320x240(均为8的倍数)

测试列表:
  ST7701默认init        <- 亚博例程用法(不带宽高), 最大嫌疑
  ST7701 640x480        <- 亚博OpenCV教程写法
  HX8399默认init
  ILI9806默认init
哪个阶段屏幕出现正确红块+可读白字, 那个配置就是答案

操作: 冷启动 -> 连IDE -> 运行本脚本 -> 盯屏幕记录哪个阶段正常
"""
import utime, os, image
from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor, CAM_CHN_ID_0

SHOW_MS = 6000  # 每个配置显示时长(毫秒)

# (标签, 驱动常量, init宽, init高)  宽高为None表示用默认init(不传宽高)
TESTS = [
    ("ST7701-default",  Display.ST7701,  None, None),
    ("ST7701-640x480",  Display.ST7701,  640,  480),
    ("HX8399-default",  Display.HX8399,  None, None),
    ("ILI9806-default", Display.ILI9806, None, None),
]


def cleanup(sensor):
    """清理本次尝试的所有媒体资源, 给下一个配置干净环境"""
    try:
        if sensor is not None:
            sensor.stop()
    except Exception:
        pass
    try:
        Display.deinit()
        MediaManager.deinit()
    except Exception:
        pass
    utime.sleep_ms(400)


print("共{}个配置, 每个6秒: 屏幕应出现 红块+白字配置名, 找出正常的那一个".format(len(TESTS)))

for label, drv, w, h in TESTS:
    sensor = None
    print("\n>>> [{}] 显示中, 盯屏幕6秒... <<<".format(label))
    try:
        # 占位采集源 (CanMV要求MediaManager前有sensor)
        sensor = Sensor(id=2)
        sensor.reset()
        sensor.set_framesize(width=320, height=240, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

        # 画布320x240, 宽高均为8的倍数 (v1的300x300踩坑)
        buf = image.Image(320, 240, image.RGB565)
        buf.clear()
        buf.draw_rectangle(0, 0, 320, 240, color=(255, 0, 0), fill=True)
        buf.draw_string_advanced(10, 10, 28, label, (255, 255, 255))

        if w is None:
            Display.init(drv, to_ide=False)   # 默认几何
        else:
            Display.init(drv, width=w, height=h, to_ide=False)
        MediaManager.init()
        sensor.run()

        t0 = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), t0) < SHOW_MS:
            os.exitpoint()
            Display.show_image(buf)
            utime.sleep_ms(50)
        print("    [{}] 结束".format(label))
    except BaseException as e:
        print("    [{}] 异常: {}".format(label, e))
    finally:
        cleanup(sensor)

print("\n>>> 全部试完! 记录: 哪个阶段屏幕是正常红块+可读配置名 (或者全部异常/全部条纹) <<<")
