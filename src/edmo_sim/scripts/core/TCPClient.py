import socket
import threading
import time

from core import stopThreading
from core.TCPCommunication import TCPCommunication

class TCPClient(TCPCommunication):

    def connect(self, host: str, port: int):
        self.tcp_socket = None
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.connect((host, port))
            self.send_text("Connection established")
            self.tcp_socket = server_socket


            self.client_th = threading.Thread(target=self.tcp_client_concurrency)
            self.client_th.daemon = True
            self.client_th.start()
            msg = 'TCP server are listening to:%s\n' % str(port)

            return True
        except ConnectionRefusedError:
            print("Could not establish a connection")
            return False

    def tcp_client_concurrency(self):
        while True:
            try:
                recv_msg = self.tcp_socket.recv(8096)
                recv_msg = recv_msg.decode('utf-8')

                if recv_msg:
                    for msg in recv_msg.split(";;;"):
                        if msg:
                            print(f"Received msg: {msg}")
                            self.broadcast_received_msg(None, None, msg)
            except ConnectionResetError:
                print(f"The connection to server has been terminated")
                self.tcp_socket.close()
            except BlockingIOError as ret:
                pass # Hacky as it always occurs

    def close_connection(self):
        if self.tcp_socket is not None:
            self.tcp_socket.close()

    def send_binary(self, msg: bytes):
        if self.tcp_socket is not None:
            self.tcp_socket.sendall(msg)
        else:
            print("Trying to send a message without established connection")

    def connection_established(self) -> bool:
        return bool(self.tcp_socket and not self.tcp_socket.is_closed())