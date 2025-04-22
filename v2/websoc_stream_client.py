# udp_stream_client.py
import cv2
import socket
import pyaudio
import threading
import pickle
import numpy as np

# ==== CONFIG ====
VIDEO_PORT = 5005
AUDIO_PORT = 5006

# ==== AUDIO CONFIG ====
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
audio = pyaudio.PyAudio()

# ==== VIDEO SOCKET ====
video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_socket.bind(('127.0.0.1', VIDEO_PORT))

# ==== AUDIO SOCKET ====
audio_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
audio_socket.bind(('127.0.0.1', AUDIO_PORT))

# ==== AUDIO PLAYBACK ====
def receive_audio():
    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, output=True, frames_per_buffer=CHUNK)
    print("[AUDIO] Receiving audio...")
    while True:
        try:
            data, _ = audio_socket.recvfrom(2048)
            stream.write(data)
        except Exception as e:
            print(f"[AUDIO ERROR] {e}")                                                                                                                                 

# ==== VIDEO DISPLAY ====
def receive_video():
    print("[VIDEO] Receiving video...")
    while True:
        try:
            packet, _ = video_socket.recvfrom(65507)
            frame_data = pickle.loads(packet)
            frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow("Live Stream", frame)
            if cv2.waitKey(1) == 27:  # ESC to quit
                break
        except Exception as e:
            print(f"[VIDEO ERROR] {e}")
    cv2.destroyAllWindows()

# ==== START THREADS ====
if __name__ == "__main__":
    threading.Thread(target=receive_audio, daemon=True).start()
    receive_video()
