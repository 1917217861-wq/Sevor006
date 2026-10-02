"""
STM32 ↔ K230 通信层 (Python版)
解析 STM32 发来的 5字节指令帧: BB + type + data + BCC + 55
BCC = type ^ data

指令类型 (与 STM32 k230_comm.h 一致):
  type=0x01 切换工作模式, data=WorkMode_t
  type=0x02 暂停/恢复, data=0(恢复)/1(暂停)
  type=0x03 激光开关, data=0(关)/1(开)
"""
from config import (
    STM32_FRAME_HEADER, STM32_FRAME_TAIL, STM32_FRAME_LEN,
    STM32_CMD_MODE, STM32_CMD_PAUSE, STM32_CMD_LASER,
    MODE_IDLE,
)


def calc_bcc(type_byte, data_byte):
    """BCC = type ^ data"""
    return (type_byte ^ data_byte) & 0xFF


class STM32Comm:
    """STM32 通信类

    回调函数 (可选, 设置后对应事件触发时调用):
        on_mode_change(mode): 工作模式切换
        on_pause(pause): 暂停/恢复, pause=1暂停/0恢复
        on_laser(on): 激光开关, on=1开/0关
    """

    def __init__(self, uart):
        """初始化
        Args:
            uart: 已配置好的 UART 对象 (K230 CanMV UART)
        """
        self.uart = uart
        self._buf = bytearray()

        # 当前状态快照
        self.current_mode = MODE_IDLE
        self.pause_flag = 0
        self.laser_state = 0

        # 回调 (由 main.py 注册)
        self.on_mode_change = None
        self.on_pause = None
        self.on_laser = None

    def poll(self):
        """非阻塞轮询: 读取并解析所有完整帧
        应在主循环中反复调用
        兼容性: 亚博固件bytearray不支持 find(int)/pop()/del buf[:i],
        帧头用手动扫描, 缓冲裁剪用切片重赋值
        """
        n = self.uart.any()
        if n == 0:
            return
        data = self.uart.read(n)
        if data is None:
            return
        self._buf.extend(data)

        # 解析所有完整帧
        while True:
            buf = self._buf
            # 手动扫描找帧头0xBB (此固件find()不支持int参数)
            idx = -1
            for i in range(len(buf)):
                if buf[i] == STM32_FRAME_HEADER:
                    idx = i
                    break
            if idx < 0:
                # 无帧头, 清空缓冲
                self._buf = bytearray()
                return
            if idx > 0:
                # 丢弃帧头前的无效字节 (切片代替del)
                buf = buf[idx:]
                self._buf = buf
            if len(buf) < STM32_FRAME_LEN:
                return  # 数据不足, 等下次

            frame = buf[:STM32_FRAME_LEN]
            # 检查帧尾 + BCC
            if frame[4] != STM32_FRAME_TAIL or calc_bcc(frame[1], frame[2]) != frame[3]:
                # 不是有效帧, 跳过这个0xBB, 继续找下一个
                self._buf = buf[1:]
                continue

            # 有效帧, 从缓冲区移除
            self._buf = buf[STM32_FRAME_LEN:]
            self._dispatch(frame[1], frame[2])

    def _dispatch(self, type_byte, data_byte):
        """分发指令到回调 (mode去重: 只在变化时回调, 防止STM32心跳包导致轨迹反复重启)"""
        if type_byte == STM32_CMD_MODE:
            mode = data_byte
            if mode != self.current_mode:   # 🔧 去重: 只在mode变化时触发
                self.current_mode = mode
                if self.on_mode_change:
                    self.on_mode_change(mode)
        elif type_byte == STM32_CMD_PAUSE:
            if data_byte != self.pause_flag:  # 🔧 去重: 只在pause状态变化时触发
                self.pause_flag = data_byte
                if self.on_pause:
                    self.on_pause(data_byte)
        elif type_byte == STM32_CMD_LASER:
            if data_byte != self.laser_state:  # 🔧 去重: 只在laser状态变化时触发
                self.laser_state = data_byte
                if self.on_laser:
                    self.on_laser(data_byte)

    def send_feedback(self, type_byte, data_bytes=b''):
        """K230 → STM32 反馈帧: AA + type + data(n) + BCC + 55
        简化格式 (Cycle 2+ 用, 当前STM32端解析器已预留)
        Args:
            type_byte: 反馈类型
            data_bytes: 数据字段
        """
        bcc = type_byte
        for b in data_bytes:
            bcc ^= b
        frame = bytes([0xAA, type_byte]) + bytes(data_bytes) + bytes([bcc & 0xFF, 0x55])
        self.uart.write(frame)
