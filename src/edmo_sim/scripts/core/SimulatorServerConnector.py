import json
from time import sleep

from core.TCPClient import TCPClient


class SimulatorServerConnector():
    def __init__(self):
        self.tcp_client = TCPClient()

    def connect(self, host="localhost", port=65343):
        connection_successful = self.tcp_client.connect(host, port)

        if not connection_successful:
            print("Connection to server could not be established")
            return

    def register_rcv_listener(self, fnct):
        def callback(host, port, msg):
            fnct(json.loads(msg))
        self.tcp_client.add_received_message_listener(callback)

    def respond_result(self, sim_task_id, result):
        self.tcp_client.send_text(json.dumps({'sim_task_id': sim_task_id, 'fitness': result}))