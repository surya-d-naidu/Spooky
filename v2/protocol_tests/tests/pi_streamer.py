# streaming_framework/pi_streamer.py
import argparse
from utils.modules.Video import VideoTransmitter
from ..utils.modules.Audio import AudioTransmitter
from ..utils.modules.Messenger import MessageReceiver

def on_message(msg, addr):
    print(f"[MSG from {addr}] {msg}")

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--laptop-ip", required=True)
    p.add_argument("--video-port", default=5005, type=int)
    p.add_argument("--audio-port", default=5006, type=int)
    p.add_argument("--msg-port",   default=5007, type=int)
    args = p.parse_args()

    # start video & audio TX
    vt = VideoTransmitter(args.laptop_ip, args.video_port)
    at = AudioTransmitter(args.laptop_ip, args.audio_port)
    vt.start(); at.start()

    # receive messages from laptop
    mr = MessageReceiver(listen_port=args.msg_port, on_message=on_message)
    mr.start()

    print("[PI] Streaming video/audio. Listening for messages.")
    try:
        while True: pass
    except KeyboardInterrupt:
        vt.stop(); at.stop(); mr.stop()
