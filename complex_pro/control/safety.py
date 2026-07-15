from complex_project.hardware.buzzer import Buzzer

class SafetyMonitor:
    def __init__(self, sensors):
        self.sensors = sensors
        self.last = None
        self.buzzer = Buzzer()  # 新增蜂鸣器实例

    def check(self, enable_safety=True):
        self.last = self.sensors.snapshot()
        # 安全开关开启时才更新蜂鸣器
        if enable_safety:
            self.buzzer.update(self.last['level'])
        else:
            # 关闭安全时强制静音，且emergency置为False
            self.last['emergency'] = False
            self.buzzer.update('safe')
        return self.last

    def close(self):
        self.buzzer.close()
