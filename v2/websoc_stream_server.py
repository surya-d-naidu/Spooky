# udp_gui_server.py
import tkinter as tk
from threading import Thread
import socket
import cv2
import pickle
import pyaudio

# ==== CONFIG ====
CLIENT_IP = '127.0.0.1'
VIDEO_PORT = 5005
AUDIO_PORT = 5006
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100

# ==== AUDIO SETUP ====
audio = pyaudio.PyAudio()
audio_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==== VIDEO SETUP ====
video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cap = None
audio_stream = None

# ==== Control flags ====
streaming = False

# ==== AUDIO THREAD ====
def stream_audio():
    global audio_stream
    audio_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    while streaming:
        try:
            data = audio_stream.read(CHUNK, exception_on_overflow=False)
            audio_socket.sendto(data, (CLIENT_IP, AUDIO_PORT))
        except:
            break
    audio_stream.stop_stream()
    audio_stream.close()

# ==== VIDEO THREAD ====
def stream_video():
    global cap
    cap = cv2.VideoCapture(0)
    cap.set(3, 320)
    cap.set(4, 240)
    while streaming:
        ret, frame = cap.read()
        if not ret:
            continue
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
        data = pickle.dumps(buffer)
        if len(data) < 65507:
            video_socket.sendto(data, (CLIENT_IP, VIDEO_PORT))
    cap.release()

# ==== GUI START ====
def start_stream():
    global streaming
    if not streaming:
        streaming = True
        Thread(target=stream_audio, daemon=True).start()
        Thread(target=stream_video, daemon=True).start()
        status_label.config(text="🟢 Streaming...")

def stop_stream():
    global streaming
    streaming = False
    status_label.config(text="🔴 Stopped")

# ==== TKINTER GUI ====
root = tk.Tk()
root.title("UDP Webcam + Audio Streamer")

tk.Label(root, text="UDP Streaming Server", font=("Arial", 16)).pack(pady=10)
start_btn = tk.Button(root, text="Start Streaming", font=("Arial", 12), command=start_stream)
stop_btn = tk.Button(root, text="Stop Streaming", font=("Arial", 12), command=stop_stream)
status_label = tk.Label(root, text="🔴 Stopped", font=("Arial", 12))

start_btn.pack(pady=5)
stop_btn.pack(pady=5)
status_label.pack(pady=5)

root.mainloop()
