from pathlib import Path
ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / 'assets'
MODELS = ASSETS / 'models'
CASCADES = ASSETS / 'cascades'
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
FRAME_CENTER = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
FACE_CASCADE = CASCADES / 'haarcascade_frontalface_default.xml'
YOLO_MODEL = MODELS / 'yolov8n320.onnx'   # 320×320 输入版，帧率翻倍；行人检测够用
YOLO_INPUT_SIZE = 320                       # 与模型对应，blob 尺寸和坐标换算分母
# 底盘通道沿用实验参考代码；使用前须以实际接线为准复核。
MOTOR_CONFIG = {
    0: {'pwm': 0, 'dir1': 2, 'dir2': 1},
    1: {'pwm': 5, 'dir1': 3, 'dir2': 4},
    2: {'pwm': 6, 'dir1': 8, 'dir2': 7},
    3: {'pwm': 11, 'dir1': 25, 'dir2': 24},
}
HORIZONTAL_CH, VERTICAL_CH = 10, 9
HORIZONTAL_LIMIT = (10, 170)
VERTICAL_LIMIT = (10, 70)
# 云台预设角度：务必先用 gimbal_manual 模式实测标定，方向随舵机安装可能相反。
HORIZONTAL_CENTER = 90   # 水平朝正前方
VERTICAL_LINE = 35       # 垂直“看地面巡线”角度（越大越低头，需实测）
VERTICAL_FACE = 15       # 垂直“看人/人脸”角度（越小越抬头，需实测）
# 全自动模式开机找人搜索超时（秒）；超时无目标转入巡线
SEARCH_TIMEOUT = 8.0

# ===== PID + 麦克纳姆连续控制参数（保守起步，实车整定）=====
# 巡线转向 PID（误差归一化 -1~1，输出 ω 归一化 -1~1）
PID_LINE_KP = 1.2
PID_LINE_KI = 0.0
PID_LINE_KD = 0.15
LINE_BASE_VX = 0.6       # 巡线基础前进速度（0~1，占全速比例）
LINE_MIN_VX = 0.25       # 急弯时最低前进速度
LINE_CURVE_GAIN = 0.8    # 偏差越大降速越多，0~1

# 行人跟随：水平偏差 PID（输出 ω，归一化 -1~1）
PID_YAW_KP = 0.8
PID_YAW_KI = 0.0
PID_YAW_KD = 0.1
# 行人跟随：距离偏差 PID（输出 vx，归一化 -1~1）
PID_DIST_KP = 0.5
PID_DIST_KI = 0.0
PID_DIST_KD = 0.05
PERSON_TARGET_W = 160    # 目标框宽度（像素），对应理想跟随距离
PERSON_MAX_VX = 0.55
PERSON_SEARCH_OMEGA = 0.5  # 搜索时原地转向速度
PERSON_DETECT_INTERVAL = 3  # KCF 跟踪模式下，每隔多少帧跑一次 YOLO 检测校准

# 云台人脸追踪 PID（误差归一化 -1~1，输出角度增量°）
PID_GIMBAL_KP = 25.0
PID_GIMBAL_KI = 0.0
PID_GIMBAL_KD = 2.0

# 麦克纳姆逆运动学
MECANUM_K = 1.0          # 转向灵敏度系数，与轮距/轴距相关，实车标定
MECANUM_BASE_SPEED = 60  # 全速对应 PWM 占空比 (%)

OBSTACLE_STOP_CM = 20.0
# 避障分级阈值（单位：cm）
OBSTACLE_WARN_CM = 40.0
OBSTACLE_DANGER_CM = 20.0
# 蜂鸣器配置
BUZZER_PIN = 17
# 三级报警参数：(频率Hz, 鸣响时长s, 停止时长s)
BUZZER_WARN = (220, 0.1, 0.5)
BUZZER_DANGER = (350, 0.1, 0.2)
BUZZER_EMERGENCY = (400, 0.2, 0.1)
