import pickle
from messages import *


class RemoteExecutive():

    def __init__(self, socket, simulator):
        self.socket = socket
        self.simulator = simulator

    def initialize(self, services):
        self.socket.send_one_message(INITILIZE_EXECUTIVE)
        return self.wait_for_one_message_and_serve_perception()

    def next_action(self):
        self.socket.send_int(NEXT_ACTION)
        return self.wait_for_one_message_and_serve_perception()

    def wait_for_one_message_and_serve_perception(self):
        while True:
            message = self.socket.recv_one_message()
            if message == PERCEPTION_REQUEST:
                self.socket.send_one_message(
                    pickle.dumps(self.simulator.perceive_state()))
            else:
                return message
