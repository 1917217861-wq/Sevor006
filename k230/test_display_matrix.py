"""
显示分辨率矩阵测试: 摸清亚博K230这块屏到底支持哪种几何
====================================================
依次测3种配置, 每种画满屏纯色6秒, 盯屏幕记录哪一阶段全屏变色:
  Phase 1: 默认init(不指定宽高) + 480x800 红色画布
  Phase 2: init(800,480)        + 800x480 蓝色画布
  Phase 3: init(480,320)        + 480x320 绿色画布

先冷启动(拔电重插)再连IDE运行本脚本
"""
import utime, os
from media.sensor import *
from media.display import *
from media.media import *
import image

SHOW_MS = 6000


def cleanup(sensor):
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


def phase(name, cw, ch, init_w, init_h, color):
    print(">>> {}: 画布 {}x{}, init {}x{}, 看屏幕 6 秒 <<<".format(
        name, cw, ch, init_w, init_h))
    sensor = None
    try:
        sensor = Sensor(id=2)
        sensor.reset()
        sensor.set_framesize(width=320, height=240, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

        if init_w is None:
            Display.init(Display.ST7701, to_ide=False)
        else:
            Display.init(Display.ST7701, width=init_w, height=init_h, to_ide=False)
        MediaManager.init()
        sensor.run()

        buf = image.Image(cw, ch, image.RGB565)
        buf.clear()
        buf.draw_rectangle(0, 0, cw, ch, color=color, thickness=-1, fill=True)
        buf.draw_string_advanced(cw // 2 - 150, ch // 2, 60, name, (255, 255, 255))

        t0 = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), t0) < SHOW_MS:
            os.exitpoint()
            Display.show_image(buf)
            utime.sleep_ms(50)
        print("    [{}] 结束".format(name))
    except BaseException as e:
        print("    [{}] 异常: {}".format(name, e))
    finally:
        cleanup(sensor)


# ---- 主流程 ----
try:
    phase("Phase1-默认init", 480, 800, None, None, (255, 0, 0))
    phase("Phase2-800x480", 800, 480, 800, 480, (0, 0, 255))
    phase("Phase3-480x320", 480, 320, 480, 320, (0, 255, 0))
    print(">>> 全部测完! 告诉AI每个阶段屏幕是什么颜色/什么现象 <<<")
except BaseException as e:
    print("[矩阵] 异常:", e)
print("[矩阵] done")
