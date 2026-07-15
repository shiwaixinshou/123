from complex_project.config import OBSTACLE_STOP_CM, OBSTACLE_WARN_CM, OBSTACLE_DANGER_CM


class SafetySensors:
    def __init__(self):
        from gpiozero import Button, DistanceSensor
        self.left = Button(12, pull_up=True)
        self.right = Button(16, pull_up=True)
        self.ultrasonic = DistanceSensor(echo=21, trigger=20, max_distance=3)

    def snapshot(self):
        distance = self.ultrasonic.distance * 100
        infrared = bool(self.left.value or self.right.value)
        
        # 分级判定：emergency(红外或≤停止阈值) > danger(≤危险) > warn(≤预警) > safe
        if infrared:
            level = 'emergency'
        elif distance <= OBSTACLE_DANGER_CM:
            level = 'danger'
        elif distance <= OBSTACLE_WARN_CM:
            level = 'warn'
        else:
            level = 'safe'

   

        return {'distance_cm': distance, 'infrared': infrared,
                'emergency': infrared or distance <= OBSTACLE_STOP_CM,'level': level }

    def close(self):
        self.left.close(); self.right.close(); self.ultrasonic.close()
