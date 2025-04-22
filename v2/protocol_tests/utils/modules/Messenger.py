# streaming_framework/message.py
from ..base.Network import UdpSender, UdpReceiver

class MessageTransmitter:
    def __init__(self, target_ip, target_port, encoding='utf-8'):
        self.sender = UdpSender(target_ip, target_port)
        self.enc = encoding

    def send(self, text: str):
        self.sender.send(text.encode(self.enc))

    def close(self):
        self.sender.close()

class MessageReceiver:
    def __init__(self, listen_port, on_message, encoding='utf-8'):
        """
        on_message(text: str, addr) -> None
        """
        self.on_message = on_message
        self.dec = encoding
        self.receiver = UdpReceiver(listen_port=listen_port)

    def start(self):
        def _handle(data, addr):
            try:
                text = data.decode(self.dec)
                self.on_message(text, addr)
            except:
                pass
        self.receiver.start(_handle)

    def stop(self):
        self.receiver.stop()
