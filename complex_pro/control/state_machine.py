import time
from enum import Enum, auto


class MissionState(Enum):
    BOOT_SCAN = auto()      # 开机找人：云台在看人区间水平扫描，车身不动
    LINE_FOLLOW = auto()
    FACE_TRACK = auto()
    PERSON_FOLLOW = auto()
    RETURN_LINE = auto()
    EMERGENCY_STOP = auto()


class MissionStateMachine:
    """全自动任务状态机（方案B）：开机先找人，超时无目标转巡线；
    之后专心巡线，中途来人不再响应；人脸/人员任务结束后经 RETURN_LINE 回巡线。
    优先级：紧急停止 > 行人跟随 > 人脸追踪 > 巡线 > 开机搜索。"""

    def __init__(self, search_timeout=8.0):
        self.state = MissionState.BOOT_SCAN
        self.search_timeout = search_timeout
        self.scan_start = time.monotonic()
        self.missing_frames = 0
        self.face_missing_frames = 0

    def update(self, safety, faces, person, line):
        # 1) 最高优先级：障碍/异常强制急停
        if safety['emergency']:
            self.state = MissionState.EMERGENCY_STOP
            return self.state
        # 2) 急停解除：方案B 下直接回巡线（不重新开机找人）
        if self.state == MissionState.EMERGENCY_STOP:
            self.state = MissionState.LINE_FOLLOW

        # 3) 各状态内部转移
        if self.state == MissionState.BOOT_SCAN:
            if person:
                self.state = MissionState.PERSON_FOLLOW
            elif faces:
                self.state = MissionState.FACE_TRACK
            elif time.monotonic() - self.scan_start > self.search_timeout:
                self.state = MissionState.LINE_FOLLOW
        elif self.state == MissionState.LINE_FOLLOW:
            # 方案B：巡线阶段不再主动找人（云台看地，也基本检测不到人）
            pass
        elif self.state == MissionState.FACE_TRACK:
            self.face_missing_frames = 0 if faces else self.face_missing_frames + 1
            if person:
                self.state = MissionState.PERSON_FOLLOW
            elif self.face_missing_frames > 50:
                self.state = MissionState.RETURN_LINE
        elif self.state == MissionState.PERSON_FOLLOW:
            self.missing_frames = 0 if person else self.missing_frames + 1
            if self.missing_frames > 20:
                self.state = MissionState.RETURN_LINE
        elif self.state == MissionState.RETURN_LINE and line.found:
            self.state = MissionState.LINE_FOLLOW
        return self.state
