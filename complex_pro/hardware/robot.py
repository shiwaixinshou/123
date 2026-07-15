import math
import time
from enum import Enum
from complex_project.config import MOTOR_CONFIG, MECANUM_K, MECANUM_BASE_SPEED


class Direction(Enum):
    FORWARD = 1
    BACKWARD = -1


class PCA9685:
    MODE1, PRESCALE, LED0_ON_L = 0x00, 0xFE, 0x06

    def __init__(self, address=0x40):
        import smbus
        self.bus = smbus.SMBus(1)
        self.address = address
        self.write(self.MODE1, 0x00)

    def write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def frequency(self, hz=50):
        prescale = math.floor(25_000_000 / 4096 / hz - 1 + 0.5)
        old = self.read(self.MODE1)
        self.write(self.MODE1, (old & 0x7F) | 0x10)
        self.write(self.PRESCALE, prescale)
        self.write(self.MODE1, old)
        time.sleep(0.005)
        self.write(self.MODE1, old | 0x80)

    def pwm(self, channel, value):
        base = self.LED0_ON_L + channel * 4
        self.write(base, 0); self.write(base + 1, 0)
        self.write(base + 2, value & 0xFF); self.write(base + 3, value >> 8)

    def duty(self, channel, percent):
        self.pwm(channel, int(max(0, min(100, percent)) * 40.95))

    def level(self, channel, value):
        self.pwm(channel, 4095 if value else 0)


class Robot:
    """麦克纳姆底盘 — 连续速度控制。
    底层接口：set_wheels(w0..w3) 每轮独立有符号速度 (-100~100)。
    高层接口：mecanum(vx, vy, omega) 由逆运动学自动拆成 4 轮速度。"""

    def __init__(self):
        from gpiozero import LED
        self.config = MOTOR_CONFIG
        self.pwm = PCA9685()
        self.pwm.frequency(50)
        self.motor3_a = LED(self.config[3]['dir1'])
        self.motor3_b = LED(self.config[3]['dir2'])
        self.stop()

    def _motor(self, index, direction, speed):
        """驱动单个电机：方向 + 占空比 (0~100)。"""
        m = self.config[index]
        self.pwm.duty(m['pwm'], speed)
        forward = direction == Direction.FORWARD
        if index == 0:
            self.pwm.level(m['dir1'], not forward); self.pwm.level(m['dir2'], forward)
        elif index in (1, 2):
            self.pwm.level(m['dir1'], forward); self.pwm.level(m['dir2'], not forward)
        else:
            (self.motor3_a.off() if forward else self.motor3_a.on())
            (self.motor3_b.on() if forward else self.motor3_b.off())

    def set_wheels(self, w0, w1, w2, w3):
        """直接设置 4 个轮子的有符号速度百分比 (-100~100)。
        正值=前进，负值=后退，0=停转。超出范围自动限幅。"""
        speeds = [w0, w1, w2, w3]
        for i, v in enumerate(speeds):
            v = max(-100.0, min(100.0, v))
            if abs(v) < 0.5:
                self.pwm.duty(self.config[i]['pwm'], 0)  # 死区：直接停转
            else:
                self._motor(i, Direction.FORWARD if v > 0 else Direction.BACKWARD, abs(v))

    def mecanum(self, vx, vy, omega, base_speed=None):
        """麦克纳姆逆运动学：输入归一化速度 (-1~1)，输出 4 轮 PWM。
        vx: 前进 (+) / 后退 (-)
        vy: 右移 (+) / 左移 (-)
        omega: 逆时针 (+) / 顺时针 (-)，由 MECANUM_K 缩放
        base_speed: 全速对应占空比 (%)，默认读 config.MECANUM_BASE_SPEED
        """
        if base_speed is None:
            base_speed = MECANUM_BASE_SPEED
        k = MECANUM_K
        # 标准 X 型麦克纳姆逆解（符号随轮子安装方向可能需取反）
        w0 = vx - vy - omega * k   # 左前
        w1 = vx + vy + omega * k   # 右前
        w2 = vx + vy - omega * k   # 左后
        w3 = vx - vy + omega * k   # 右后

        # 归一化限幅：若有轮子超速，整体按比例缩放，保证方向关系不变
        max_w = max(abs(w0), abs(w1), abs(w2), abs(w3), 1e-6)
        if max_w > 1.0:
            w0 /= max_w; w1 /= max_w; w2 /= max_w; w3 /= max_w

        self.set_wheels(
            w0 * base_speed, w1 * base_speed,
            w2 * base_speed, w3 * base_speed,
        )

    def stop(self):
        for i in range(4):
            self.pwm.duty(self.config[i]['pwm'], 0)

    def close(self):
        self.stop()
        self.motor3_a.close(); self.motor3_b.close()
