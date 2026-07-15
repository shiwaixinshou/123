import time
from complex_project.control.pid import PID
from complex_project.config import (
    FRAME_WIDTH, FRAME_HEIGHT, FRAME_CENTER,
    HORIZONTAL_CH, VERTICAL_CH,
    HORIZONTAL_LIMIT, VERTICAL_LIMIT,
    HORIZONTAL_CENTER, VERTICAL_LINE, VERTICAL_FACE,
    PID_GIMBAL_KP, PID_GIMBAL_KI, PID_GIMBAL_KD,
)


class Gimbal:
    """双舵机云台 — PID 连续追踪 + 限位保护 + 水平扫描。"""

    def __init__(self):
        from adafruit_servokit import ServoKit
        self.kit = ServoKit(channels=16)
        # 默认开机先看地面（巡线基准）；各模式会再调 look_person / look_ground
        self.horizontal, self.vertical = HORIZONTAL_CENTER, VERTICAL_LINE
        self.last_update, self.scan_direction = 0.0, 1
        # 水平/垂直各一个 PID：误差归一化 -1~1，输出角度增量（度）
        self.pid_h = PID(PID_GIMBAL_KP, PID_GIMBAL_KI, PID_GIMBAL_KD, limit=8.0)  # 单帧最大±8°
        self.pid_v = PID(PID_GIMBAL_KP, PID_GIMBAL_KI, PID_GIMBAL_KD, limit=6.0)  # 垂直范围小，限幅更小
        self._apply()

    def _apply(self):
        self.kit.servo[HORIZONTAL_CH].angle = self.horizontal
        self.kit.servo[VERTICAL_CH].angle = self.vertical

    def _clamp(self):
        self.horizontal = max(HORIZONTAL_LIMIT[0], min(HORIZONTAL_LIMIT[1], self.horizontal))
        self.vertical = max(VERTICAL_LIMIT[0], min(VERTICAL_LIMIT[1], self.vertical))

    def track(self, center, dt=0.05):
        """PID 连续追踪：输入目标中心像素坐标，输出平滑角度调整。
        dt 建议取帧间实际时间，缺省按 20 FPS 估算。"""
        # 误差归一化到 -1~1（右偏/下偏为正）
        err_h = (center[0] - FRAME_CENTER[0]) / (FRAME_WIDTH / 2)
        err_v = (center[1] - FRAME_CENTER[1]) / (FRAME_HEIGHT / 2)

        delta_h = self.pid_h.update(err_h, dt)
        delta_v = self.pid_v.update(err_v, dt)

        self.horizontal += delta_h
        self.vertical += delta_v
        self._clamp()
        self._apply()

    def reset_pid(self):
        """切换目标/丢失重获时调用，避免积分项和微分项跳变。"""
        self.pid_h.reset()
        self.pid_v.reset()

    def scan(self, step=2, interval=0.12):
        """水平往返扫描（垂直角度保持不变，用于在看人区间搜索目标）。"""
        if time.monotonic() - self.last_update < interval: return
        self.horizontal += self.scan_direction * step
        if self.horizontal <= HORIZONTAL_LIMIT[0] or self.horizontal >= HORIZONTAL_LIMIT[1]:
            self.scan_direction *= -1
        self._clamp()
        self._apply(); self.last_update = time.monotonic()

    def set_angle(self, horizontal=None, vertical=None):
        """手动设置绝对角度，带软件限位（供 gimbal_manual 与滑块使用）。"""
        if horizontal is not None: self.horizontal = horizontal
        if vertical is not None: self.vertical = vertical
        self._clamp()
        self._apply()

    def look_ground(self):
        """转到巡线看地角度：水平回正，垂直看地面。"""
        self.set_angle(horizontal=HORIZONTAL_CENTER, vertical=VERTICAL_LINE)
        self.reset_pid()

    def look_person(self):
        """转到看人角度：水平回正，垂直抬到人脸高度。"""
        self.set_angle(horizontal=HORIZONTAL_CENTER, vertical=VERTICAL_FACE)
        self.reset_pid()

    def reset(self):
        """复位到巡线看地基准角度。"""
        self.look_ground()

    def close(self):
        """退出时回正到看地基准并释放（舵机无强制释放接口，异常忽略）。"""
        try:
            self.look_ground()
        except Exception:
            pass
