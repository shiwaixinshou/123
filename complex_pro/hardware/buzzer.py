import time
from gpiozero import TonalBuzzer
from complex_project.config import (
    BUZZER_PIN, BUZZER_WARN, BUZZER_DANGER, BUZZER_EMERGENCY
)

class Buzzer:
    def __init__(self):
        self.bz = TonalBuzzer(BUZZER_PIN)
        self.current_level = 'safe'
        self.is_on = False
        self.last_toggle = 0.0
        self._param_map = {
            'warn': BUZZER_WARN,
            'danger': BUZZER_DANGER,
            'emergency': BUZZER_EMERGENCY
        }

    def update(self, level):
        """主循环每帧调用，非阻塞更新蜂鸣器状态"""
        # 安全状态：直接静音
        if level == 'safe':
            if self.is_on:
                self.bz.stop()
                self.is_on = False
            self.current_level = level
            return

        # 等级变化时重置状态
        if level != self.current_level:
            self.bz.stop()
            self.is_on = False
            self.last_toggle = time.monotonic()
            self.current_level = level

        freq, on_time, off_time = self._param_map[level]
        now = time.monotonic()

        # 按时间切换鸣响/停止
        if self.is_on and now - self.last_toggle >= on_time:
            self.bz.stop()
            self.is_on = False
            self.last_toggle = now
        elif not self.is_on and now - self.last_toggle >= off_time:
            self.bz.play(freq)
            self.is_on = True
            self.last_toggle = now

    def close(self):
        """释放资源"""
        self.bz.stop()
        self.bz.close()
