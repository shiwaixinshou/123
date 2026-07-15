from dataclasses import dataclass
import cv2
import numpy as np
from complex_project.control.pid import PID
from complex_project.config import (
    PID_LINE_KP, PID_LINE_KI, PID_LINE_KD,
    LINE_BASE_VX, LINE_MIN_VX, LINE_CURVE_GAIN,
)


@dataclass
class LineResult:
    found: bool
    error: float = 0.0
    sharp_turn: bool = False
    mask: object = None


class LineFollower:
    """视觉巡线：ROI 二值化 + 轮廓 + 质心偏差，输出连续 (vx, omega)。"""

    def __init__(self):
        self.pid = PID(PID_LINE_KP, PID_LINE_KI, PID_LINE_KD, limit=1.0)

    def detect(self, frame):
        h, w = frame.shape[:2]; y0 = int(h * .62); roi = frame[y0:h]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return LineResult(False, mask=mask)
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 350: return LineResult(False, mask=mask)
        m = cv2.moments(c)
        if not m['m00']: return LineResult(False, mask=mask)
        cx = m['m10'] / m['m00']; error = (cx - w / 2) / (w / 2)
        x, _, bw, _ = cv2.boundingRect(c)
        return LineResult(True, error, abs(error) > .45 or bw < w*.18, mask)

    def control(self, result, dt):
        """返回归一化 (vx, omega)，供 robot.mecanum 直接调用。
        - 未找到线：原地右转搜索
        - 找到线：PID 输出 omega，vx 随 |error| 连续降速（急弯自动减速）
        """
        if not result.found:
            self.pid.reset()
            return 0.0, 0.6  # 原地顺时针搜索（omega>0 为逆时针，这里用正号，方向不对调 MECANUM_K 符号）

        omega = self.pid.update(result.error, dt)
        # 偏差越大，前进速度越低 → 急弯自然降速
        vx = LINE_BASE_VX * (1.0 - LINE_CURVE_GAIN * abs(result.error))
        vx = max(LINE_MIN_VX, vx)
        return vx, omega

    def reset(self):
        self.pid.reset()
