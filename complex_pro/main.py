"""集成入口。按 q 安全退出、r 复位云台；启动前将模型与级联文件放入 assets 对应目录。
底盘控制：PID + 麦克纳姆逆运动学连续速度控制（vx, vy, omega）。"""
import time
import cv2
import tkinter as tk
from tkinter import ttk
from complex_project.config import FACE_CASCADE, YOLO_MODEL, SEARCH_TIMEOUT
from complex_project.hardware.robot import Robot
from complex_project.hardware.gimbal import Gimbal
from complex_project.hardware.sensors import SafetySensors
from complex_project.vision.camera import Camera
from complex_project.vision.face_tracker import FaceTracker
from complex_project.vision.person_tracker import PersonTracker
from complex_project.vision.line_follower import LineFollower
from complex_project.control.safety import SafetyMonitor
from complex_project.control.state_machine import MissionStateMachine, MissionState

WINDOW = 'Indoor Inspection Robot'


def show_mode_selector():
    """启动模式选择窗口，返回用户选择的配置。"""
    root = tk.Tk()
    root.title("功能选择")
    root.geometry("320x400")
    root.resizable(False, False)

    selected_mode = tk.StringVar(value="full_auto")
    safety_enabled = tk.BooleanVar(value=True)

    ttk.Label(root, text="请选择运行模式：", font=("Arial", 11, "bold")).pack(anchor="w", padx=20, pady=(15, 8))
    modes = [
        ("完整自动模式", "full_auto"),
        ("纯巡线测试", "line_only"),
        ("纯人脸追踪测试", "face_only"),
        ("纯行人跟随测试", "person_only"),
        ("云台手动调试", "gimbal_manual"),
    ]
    for text, value in modes:
        ttk.Radiobutton(root, text=text, variable=selected_mode, value=value).pack(anchor="w", padx=30, pady=2)

    ttk.Checkbutton(
        root, text="启用安全保护（紧急停止+蜂鸣器）", variable=safety_enabled
    ).pack(anchor="w", padx=20, pady=(15, 10))

    result = {}
    def on_start():
        result['mode'] = selected_mode.get()
        result['safety'] = safety_enabled.get()
        root.destroy()

    ttk.Button(root, text="开始运行", command=on_start, width=20).pack(pady=20)
    root.mainloop()
    return result


def main():
    config = show_mode_selector()
    if not config:  # 用户直接关闭选择窗
        return
    run_mode = config['mode']
    safety_on = config['safety']

    camera = robot = gimbal = sensors = safety = None
    face = person = line = mission = None
    prev_state = None
    last_t = time.monotonic()

    try:
        # ========== 按需初始化 ==========
        camera = Camera()          # 所有模式都需要摄像头
        gimbal = Gimbal()          # 所有模式都带云台（巡线也要摆到看地角度）

        if run_mode in ['full_auto', 'line_only', 'person_only']:
            robot = Robot()
            sensors = SafetySensors()       # 始终创建，安全是否生效由 enable_safety 控制
            safety = SafetyMonitor(sensors)
        if run_mode in ['full_auto', 'face_only']:
            face = FaceTracker(FACE_CASCADE)
        if run_mode in ['full_auto', 'person_only']:
            person = PersonTracker(YOLO_MODEL)
        if run_mode in ['full_auto', 'line_only']:
            line = LineFollower()
        if run_mode == 'full_auto':
            mission = MissionStateMachine(search_timeout=SEARCH_TIMEOUT)

        # 各模式初始云台角度
        if run_mode == 'line_only':
            gimbal.look_ground()
        elif run_mode in ['face_only', 'person_only']:
            gimbal.look_person()
        # full_auto 由状态机进入 BOOT_SCAN 时设置；gimbal_manual 保持默认看地

        if run_mode == 'gimbal_manual':
            cv2.namedWindow(WINDOW)
            cv2.createTrackbar('Horizontal', WINDOW, gimbal.horizontal, 180,
                               lambda v: gimbal.set_angle(horizontal=v))
            cv2.createTrackbar('Vertical', WINDOW, gimbal.vertical, 90,
                               lambda v: gimbal.set_angle(vertical=v))

        # ========== 主循环 ==========
        while True:
            frame = camera.read()
            now = time.monotonic()
            dt = max(0.001, now - last_t)
            last_t = now

            # ---------- 完整自动 ----------
            if run_mode == 'full_auto':
                safety_state = safety.check(enable_safety=safety_on)
                faces, face_center, _ = face.detect(frame)
                person_target = person.detect(frame)
                line_result = line.detect(frame)
                state = mission.update(safety_state, faces, person_target, line_result)

                # 状态切换钩子：进入某状态时一次性摆好云台角度 + 重置 PID
                if state != prev_state:
                    if state in (MissionState.BOOT_SCAN, MissionState.FACE_TRACK, MissionState.PERSON_FOLLOW):
                        gimbal.look_person()
                    elif state in (MissionState.LINE_FOLLOW, MissionState.RETURN_LINE):
                        gimbal.look_ground()
                        line.reset()
                    if state == MissionState.PERSON_FOLLOW:
                        person.reset()
                    prev_state = state

                if state == MissionState.EMERGENCY_STOP:
                    robot.stop()
                elif state == MissionState.BOOT_SCAN:
                    gimbal.scan(); robot.stop()          # 车身不动，仅云台水平扫描找人
                elif state == MissionState.LINE_FOLLOW:
                    vx, omega = line.control(line_result, dt)
                    robot.mecanum(vx, 0, omega)
                elif state == MissionState.FACE_TRACK:
                    if face_center:
                        gimbal.track(face_center, dt)
                    else:
                        gimbal.scan()
                    robot.stop()
                elif state == MissionState.PERSON_FOLLOW:
                    vx, omega = person.control(person_target, dt)
                    robot.mecanum(vx, 0, omega)
                elif state == MissionState.RETURN_LINE:
                    if line_result.found:
                        robot.stop()
                    else:
                        robot.mecanum(0, 0, 0.5)   # 原地转搜线

                face.draw(frame, faces, face_center)
                person.draw(frame, person_target)
                cv2.putText(frame, state.name, (12, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2)

            # ---------- 纯巡线 ----------
            elif run_mode == 'line_only':
                safety_state = safety.check(enable_safety=safety_on)
                line_result = line.detect(frame)
                if safety_state['emergency']:
                    robot.stop()
                else:
                    vx, omega = line.control(line_result, dt)
                    robot.mecanum(vx, 0, omega)
                cv2.putText(frame, f"LINE  err:{line_result.error:.2f}", (12, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 0), 2)

            # ---------- 纯人脸追踪（无底盘、无安全，可贴近测试） ----------
            elif run_mode == 'face_only':
                faces, face_center, _ = face.detect(frame)
                if face_center:
                    gimbal.track(face_center, dt)
                else:
                    gimbal.scan()          # 丢失目标时水平扫描搜索
                face.draw(frame, faces, face_center)
                cv2.putText(frame, "FACE TRACK", (12, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 200, 255), 2)

            # ---------- 纯行人跟随 ----------
            elif run_mode == 'person_only':
                safety_state = safety.check(enable_safety=safety_on)
                person_target = person.detect(frame)
                if safety_state['emergency']:
                    robot.stop()
                else:
                    vx, omega = person.control(person_target, dt)
                    robot.mecanum(vx, 0, omega)
                person.draw(frame, person_target)
                cv2.putText(frame, "PERSON FOLLOW", (12, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 180, 0), 2)

            # ---------- 云台手动调试（标定看人/看地角度） ----------
            elif run_mode == 'gimbal_manual':
                cv2.putText(frame, f"H:{gimbal.horizontal}  V:{gimbal.vertical}", (12, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
                cv2.putText(frame, "drag sliders / press R to reset", (12, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)

            cv2.imshow(WINDOW, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('r') and gimbal:
                gimbal.reset()

    finally:
        if robot:
            robot.close()
        if sensors:
            sensors.close()
        if safety:
            safety.close()
        if gimbal:
            gimbal.close()
        if camera:
            camera.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
