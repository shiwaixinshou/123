import time
from pathlib import Path
import cv2


class FaceTracker:
    """多人定位；面积最大且连续出现的人脸是云台主目标。"""
    def __init__(self, cascade_path: Path, lost_timeout=1.0):
        if not cascade_path.exists(): raise FileNotFoundError(f'缺少人脸级联文件: {cascade_path}')
        self.cascade = cv2.CascadeClassifier(str(cascade_path))
        if self.cascade.empty(): raise RuntimeError(f'无法加载人脸级联文件: {cascade_path}')
        self.last_center = None; self.last_seen = 0.0; self.lost_timeout = lost_timeout

    def detect(self, frame):
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        faces = [tuple(map(int, item)) for item in self.cascade.detectMultiScale(gray, 1.1, 6, minSize=(40, 40))]
        target = max(faces, key=lambda f: f[2] * f[3]) if faces else None
        if target:
            x, y, w, h = target; center = (x + w // 2, y + h // 2)
            if self.last_center: center = (int(self.last_center[0] * .6 + center[0] * .4), int(self.last_center[1] * .6 + center[1] * .4))
            self.last_center, self.last_seen = center, time.monotonic()
        else: center = None
        return faces, center, time.monotonic() - self.last_seen > self.lost_timeout

    @staticmethod
    def draw(frame, faces, target_center):
        for x, y, w, h in faces: cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 200, 0), 2)
        if target_center: cv2.circle(frame, target_center, 5, (0, 0, 255), -1)
        return frame
