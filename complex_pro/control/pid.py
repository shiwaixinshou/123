class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, limit=1.0):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.integral = 0.0; self.previous = None

    def update(self, error, dt):
        self.integral += error * dt
        derivative = 0.0 if self.previous is None or dt <= 0 else (error - self.previous) / dt
        self.previous = error
        return max(-self.limit, min(self.limit, self.kp * error + self.ki * self.integral + self.kd * derivative))

    def reset(self): self.integral = 0.0; self.previous = None
