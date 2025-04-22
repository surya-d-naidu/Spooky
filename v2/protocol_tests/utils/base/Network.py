# streaming_framework/Network.py
import socket, threading

class UdpSender:
    def __init__(self, target_ip, target_port):
        self.addr = (target_ip, target_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, data: bytes):
        self.sock.sendto(data, self.addr)

    def close(self):
        self.sock.close()

class UdpReceiver:
    def __init__(self, listen_ip='0.0.0.0', listen_port=0, buffer_size=65507):
        self.addr = (listen_ip, listen_port)
        self.buffer_size = buffer_size
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.addr)
        self.running = False

    def start(self, on_packet):
        """
        on_packet(data: bytes, addr: (ip,port)) -> None
        """
        self.running = True
        def _listen():
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(self.buffer_size)
                    on_packet(data, addr)
                except Exception as e:
                    print(f"[UDP ERROR] {e}")
                    break
        threading.Thread(target=_listen, daemon=True).start()

    def stop(self):
        self.running = False
        self.sock.close()
