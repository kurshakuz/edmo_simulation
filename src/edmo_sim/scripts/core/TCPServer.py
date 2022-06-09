import socket
import threading
import time

from core import stopThreading
from core.TCPCommunication import TCPCommunication


class TCPServer(TCPCommunication):
    def __init__(self):
        super().__init__()

        self.client_socket_list = []

    def start(self, port : int):
        print("Start TCP Server")
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_socket.setblocking(False)
        try:
            self.tcp_socket.bind(('', port))
        except Exception as ret:
            print("Please check out the port [?]")
            print(ret)
        else:
            self.tcp_socket.listen()
            self.sever_th = threading.Thread(target=self.tcp_server_concurrency)
            self.sever_th.start()
            print('TCP server are listening to:%s\n' % str(port))

    def tcp_server_concurrency(self):
        while True:
            try:
                client_socket, client_address = self.tcp_socket.accept()
            except Exception as ret:
                time.sleep(0.001)
            else:
                client_socket.setblocking(False)
                self.client_socket_list.append((client_socket, client_address))
                self.broadcast_connection_established(client_address[0], client_address[1])
                print(f"Client {client_address} connected")
            for client, address in self.client_socket_list:
                try:
                    recv_msg = client.recv(8096)
                    recv_msg = recv_msg.decode('utf-8')

                    if recv_msg:
                        for msg in recv_msg.split(";;;"):
                            if msg:
                                print(f"Received msg: {msg}")
                                self.broadcast_received_msg(address[0], address[1], msg)
                    else:
                        client.close()
                        self.client_socket_list.remove((client, address))
                except ConnectionResetError:
                    print(f"The connection to client {address[0]} {address[1]} has been terminated")
                    client.close()
                    self.client_socket_list.remove((client, address))
                except BlockingIOError as ret:
                    pass # Hacky as it always occurs


    def tcp_close(self):
        try:
            for client, address in self.client_socket_list:
                client.close()
            self.tcp_socket.close()
            print('TCP disconnect!\n')
        except Exception as ret:
            print(ret)

        try:
            stopThreading.stop_thread(self.sever_th)
        except Exception as e:
            print(e)
        try:
            stopThreading.stop_thread(self.client_th)
        except Exception as e:
            print(e)

    def send_binary(self, msg: bytes):
        for client in self.client_socket_list:
            client[0].sendall(msg)