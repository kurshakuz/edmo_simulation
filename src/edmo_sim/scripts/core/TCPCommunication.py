import time
from typing import Callable


class TCPCommunication:
    def __init__(self, debug = False):
        self.tcp_socket = None
        self.rcv_msg_handlers = []
        self.connection_established_event_handlers = []

        self.server_th = None

    def send_text(self, msg: str):
        self.send_binary(bytes(msg + ";;;", "utf-8"))

    def send_binary(self, msg: bytes):
        pass

    def add_received_message_listener(self, handler):
        self.rcv_msg_handlers.append(handler)

    def broadcast_received_msg(self, host : str, port : int, msg : str):
        for handler in self.rcv_msg_handlers:
            handler(host, port, msg)

    def add_connection_established_listener(self, handler):
        self.connection_established_event_handlers.append(handler)

    def broadcast_connection_established(self, host, port):
        for handler in self.connection_established_event_handlers:
            handler(host, port)
