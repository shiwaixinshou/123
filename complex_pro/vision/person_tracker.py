from pathlib import Path
import cv2
import numpy as np
from complex_project.control.pid import PID
from complex_project.config import (
    FRAME_WIDTH, FRAME_CENTER, YOLO_INPUT_SIZE,
    PID_YAW_KP, PID_YAW_KI, PID_YAW_KD,
    PID_DIST_KP, PID_DIST_KI, PID_DIST_KD,
    PERSON_TARGET_W, PERSON_MAX_VX, PERSON_SEARCH_OMEGA,
    PERSON_DETECT_INTERVAL,
)


class PersonTracker:
    """YOLOv8 ONNX 人员检测 + KCF 跟踪融合 + 双 PID 连续控制。
    检测-跟踪协同：每隔 DETECT_INTERVAL 帧跑一次 YOLO 校准，中间帧用 KCF 预测位置。
    优势：输出帧率高、抗短时遮挡、目标锁定稳定；折线运动由 vx+omega 同时非零自然形成。"""

    def __init__(self, model_path: Path, conf=0.5, nms=0.4):
        if not model_path.exists(): raise FileNotFoundError(f'缺少 YOLO ONNX 模型: {model_path}')
        self.net = cv2.dnn.readNetFromONNX(str(model_path)); self.conf, self.nms = conf, nms
        self.smooth = None
        self.input_size = YOLO_INPUT_SIZE

        # 水平偏差 PID：输出 omega（-1~1）
        self.pid_yaw = PID(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, limit=1.0)
        # 距离偏差 PID：输出 vx（-1~1），正=前进，负=后退
        self.pid_dist = PID(PID_DIST_KP, PID_DIST_KI, PID_DIST_KD, limit=1.0)
        self.target_w = PERSON_TARGET_W
        self.center_x = FRAME_CENTER[0]

        # KCF 跟踪器 + 帧计数（检测-跟踪协同）
        self.tracker = None
        self.tracker_score = 0.0    # 跟踪框沿用上次检测的置信度
        self.frame_count = 0
        self.detect_interval = PERSON_DETECT_INTERVAL

    # ---------- 检测核心 ----------
    def _yolo_detect(self, frame):
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1/255, (self.input_size, self.input_size), swapRB=True)
        self.net.setInput(blob); out = self.net.forward()[0].transpose()
        boxes, scores = [], []
        sz = self.input_size
        for row in out:
            cls = int(np.argmax(row[4:])); score = float(row[4 + cls])
            if cls == 0 and score >= self.conf:
                cx, cy, bw, bh = row[:4]
                boxes.append([int((cx-bw/2)*w/sz), int((cy-bh/2)*h/sz), int(bw*w/sz), int(bh*h/sz)])
                scores.append(score)
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf, self.nms)
        if len(indices) == 0:
            return None
        i = int(indices.flatten()[0]); x, y, bw, bh = boxes[i]
        candidate = (x, y, bw, bh, scores[i])
        if self.smooth:
            old = self.smooth
            candidate = tuple(int(old[j]*.6 + candidate[j]*.4) if j < 4 else candidate[j] for j in range(5))
        self.smooth = candidate
        return candidate

    # ---------- 检测 + 跟踪融合主接口 ----------
    def detect(self, frame):
        """返回目标框 (x, y, w, h, score) 或 None。
        内部自动在 YOLO 检测与 KCF 跟踪之间切换。"""
        self.frame_count += 1
        need_detect = (self.tracker is None) or (self.frame_count % self.detect_interval == 0)

        if need_detect:
            target = self._yolo_detect(frame)
            if target:
                # 检测成功 → 初始化/刷新 KCF 跟踪器
                x, y, w, h, score = target
                if self.tracker is None:
                    self.tracker = cv2.legacy.TrackerKCF_create()
                self.tracker.init(frame, (float(x), float(y), float(w), float(h)))
                self.tracker_score = score
                return target
            else:
                # 检测失败 → 跟踪器作废，下一帧继续尝试检测
                self.tracker = None
                self.smooth = None
                return None
        else:
            # 跟踪帧：用 KCF 预测位置
            if self.tracker is None:
                return None
            ok, bbox = self.tracker.update(frame)
            if ok:
                x, y, w, h = [int(v) for v in bbox]
                # 跟踪帧输出：框 + 上次检测置信度（保守标记）
                return (x, y, w, h, self.tracker_score)
            else:
                # 跟踪失败 → 立即触发下帧重新检测
                self.tracker = None
                self.frame_count = self.detect_interval - 1
                return None

    # ---------- PID 连续控制 ----------
    def control(self, target, dt):
        """返回归一化 (vx, omega)。目标为空时原地搜索。
        折线运动：vx 和 omega 同时非零 → 自然走出曲线/折线。"""
        if not target:
            self.pid_yaw.reset(); self.pid_dist.reset()
            return 0.0, PERSON_SEARCH_OMEGA   # 原地逆时针搜索

        x, y, w, h, _ = target
        # 水平偏差（归一化 -1~1）：右偏为正 → omega 正=逆时针修正
        x_error = (x + w / 2 - self.center_x) / (FRAME_WIDTH / 2)
        omega = self.pid_yaw.update(x_error, dt)

        # 距离偏差：目标框比目标小 → 太远 → 前进（vx 正）
        dist_error = (self.target_w - w) / self.target_w   # 归一化
        vx = self.pid_dist.update(dist_error, dt)
        vx = max(-PERSON_MAX_VX, min(PERSON_MAX_VX, vx))

        return vx, omega

    def reset(self):
        self.pid_yaw.reset(); self.pid_dist.reset()
        self.tracker = None
        self.smooth = None

    # ---------- 绘制 ----------
    @staticmethod
    def draw(frame, target):
        if target:
            x, y, w, h, score = target
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 180, 0), 2)
            cv2.putText(frame, f'person {score:.2f}', (x, max(20, y-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 180, 0), 2)
        return frame
