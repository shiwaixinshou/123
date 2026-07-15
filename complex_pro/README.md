# 室内智能巡检视觉小车

运行前，将 `yolov8n.onnx` 放入 `assets/models/`，将 OpenCV 的
`haarcascade_frontalface_default.xml` 放入 `assets/cascades/`。在树莓派环境安装
Picamera2、OpenCV、gpiozero、adafruit-circuitpython-servokit 与 smbus 后执行：

```bash
python -m complex_project.main
```

按 `q` 安全停止。首次实物运行前必须抬起车轮，核对电机方向、舵机通道与 GPIO 接线。

本工程是待实测的 L3 实现，不应将设计阈值当作已验证结果。
