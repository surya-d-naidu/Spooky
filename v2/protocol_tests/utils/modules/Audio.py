# streaming_framework/audio.py
import pyaudio, threading
from ..base.Network import UdpSender, UdpReceiver

class AudioTransmitter:
    def __init__(self, target_ip, target_port, rate=44100, chunk=1024, channels=1, fmt=pyaudio.paInt16):
        self.sender = UdpSender(target_ip, target_port)
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.rate = rate
        self.chunk = chunk
        self.channels = channels
        self.format = fmt
        self.running = False

    def start(self):
        self.stream = self.audio.open(format=self.format,
                                      channels=self.channels,
                                      rate=self.rate,
                                      input=True,
                                      frames_per_buffer=self.chunk)
        self.running = True
        def _tx():
            while self.running:
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                self.sender.send(data)
        threading.Thread(target=_tx, daemon=True).start()

    def stop(self):
        self.running = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()
        self.sender.close()

class AudioReceiver:
    def __init__(self, listen_port, on_chunk, rate=44100, channels=1, fmt=pyaudio.paInt16, chunk=1024):
        """
        on_chunk(data: bytes) -> None
        """
        self.on_chunk = on_chunk
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=fmt,
                                      channels=channels,
                                      rate=rate,
                                      output=True,
                                      frames_per_buffer=chunk)
        self.receiver = UdpReceiver(listen_port=listen_port)

    def start(self):
        def _handle(data, addr):
            self.on_chunk(data)
        self.receiver.start(_handle)

    def stop(self):
        self.receiver.stop()
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
