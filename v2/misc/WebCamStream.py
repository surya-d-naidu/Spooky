import socket
import threading
import cv2
import pickle
import pyaudio

# ==== CONFIG ====
CLIENT_IP = '192.168.98.216'
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

# ==== Control flag ====
streaming = False

# ==== AUDIO THREAD ====
def stream_audio():
    global audio_stream
    audio_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    print("[AUDIO] Streaming started.")
    while streaming:
        try:
            data = audio_stream.read(CHUNK, exception_on_overflow=False)
            audio_socket.sendto(data, (CLIENT_IP, AUDIO_PORT))
        except Exception as e:
            print(f"[AUDIO] Error: {e}")
            break
    audio_stream.stop_stream()
    audio_stream.close()
    print("[AUDIO] Streaming stopped.")

# ==== VIDEO THREAD ====
def stream_video():
    global cap
    cap = cv2.VideoCapture(0)
    cap.set(3, 320)
    cap.set(4, 240)
    print("[VIDEO] Streaming started.")
    while streaming:
        ret, frame = cap.read()
        if not ret:
            continue
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
        data = pickle.dumps(buffer)
        if len(data) < 65507:
            video_socket.sendto(data, (CLIENT_IP, VIDEO_PORT))
    cap.release()
    print("[VIDEO] Streaming stopped.")

# ==== START STREAM ====
def start_stream():
    global streaming
    if not streaming:
        streaming = True
        threading.Thread(target=stream_audio, daemon=True).start()
        threading.Thread(target=stream_video, daemon=True).start()
        print("🟢 Streaming...")

# ==== STOP STREAM ====
def stop_stream():
    global streaming
    streaming = False
    print("🔴 Stopped streaming.")

# ==== MAIN CLI LOOP ====
if __name__ == "__main__":
    print(f"=== UDP Webcam + Audio Streamer (CLI) === {socket.AF_INET}")
    print("Type 'start' to begin streaming, 'stop' to end, or 'exit' to quit.")

    while True:
        cmd = input(">>> ").strip().lower()
        if cmd == "start":
            start_stream()
        elif cmd == "stop":
            stop_stream()
        elif cmd == "exit":
            stop_stream()
            break
        else:
            print("Unknown command. Type 'start', 'stop', or 'exit'.")
