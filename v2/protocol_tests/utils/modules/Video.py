# streaming_framework/video.py
import cv2, pickle, threading
from ..base.Network import UdpSender, UdpReceiver

class VideoTransmitter:
    def __init__(self, target_ip, target_port, cam_index=0, width=320, height=240, quality=60):
        self.sender = UdpSender(target_ip, target_port)
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.quality = quality
        self.running = False

    def start(self):
        self.running = True
        def _tx():
            while self.running:
                ret, frame = self.cap.read()
                if not ret: continue
                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                data = pickle.dumps(buf)
                if len(data) < 65507:
                    self.sender.send(data)
        threading.Thread(target=_tx, daemon=True).start()

    def stop(self):
        self.running = False
        self.cap.release()
        self.sender.close()

class VideoReceiver:
    def __init__(self, listen_port, on_frame, window_name="Video"):
        """
        on_frame(frame: np.ndarray) -> None
        """
        self.on_frame = on_frame
        self.window = window_name
        self.receiver = UdpReceiver(listen_port=listen_port)

    def start(self):
        def _handle(data, addr):
            try:
                buf = pickle.loads(data)
                frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if frame is not None:
                    self.on_frame(frame)
            except Exception as e:
                print(f"[VIDEO ERROR] {e}")
        self.receiver.start(_handle)

    def stop(self):
        self.receiver.stop()
        cv2.destroyAllWindows()
