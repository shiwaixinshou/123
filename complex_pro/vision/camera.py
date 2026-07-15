from complex_project.config import FRAME_WIDTH, FRAME_HEIGHT


class Camera:
    def __init__(self):
        from picamera2 import Picamera2
        self.camera = Picamera2()
        config = self.camera.create_video_configuration(main={'size': (FRAME_WIDTH, FRAME_HEIGHT)})
        self.camera.configure(config); self.camera.start()

    def read(self):
        import cv2
        frame = self.camera.capture_array()
        return cv2.flip(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR), -1)

    def close(self): self.camera.stop(); self.camera.close()
