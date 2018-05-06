
import socket
import sys
import os
import struct
import pickle
from socket_utils import *
from messages import *
from tmpdir import TmpDir
from pddlsim.services.simulator_services import SimulatorServices
from pddlsim.fd_parser import FDParser
from pddlsim.services.perception import Perception


class RemoteSimulator():

    def __init__(self, host='localhost', port=9999):
        self.host = host
        self.port = port
        self.sent_domain_and_problem = False
        self.original_socket = None
        self.sock = None
        self.report_card = None

    def __enter__(self):
        self.original_socket = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.original_socket.connect((self.host, self.port))
        self.sock = BufferedSocket(self.original_socket)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.original_socket.close()

    def use_domain_and_problem(self, domain_path, problem_path):
        self.domain_path = domain_path
        self.problem_path = problem_path
        return self

    def use_planner(self, planner):
        self.planner = planner
        return self

    def get_state(self):
        self.sock.send_one_message(PERCEPTION_REQUEST)
        return pickle.loads(self.sock.recv_one_message())

    def simulate(self, executive):
        message = self.sock.recv_one_message()
        if message == SENDING_PDDLS:
            tmp_dir = TmpDir('.tmp')
            self.domain_path = tmp_dir.join('domain.pddl')
            self.problem_path = tmp_dir.join('problem.pddl')
            self.sock.get_file(self.domain_path)
            self.sock.get_file(self.problem_path)
        elif message == WAITING_FOR_PDDLS:
            self.sock.send_file(self.domain_path)
            self.sock.send_file(self.problem_path)
        else:
            return

        message = self.sock.recv_one_message()
        if message != INITILIZE_EXECUTIVE:
            return

        services = SimulatorServices.from_pddls(
            self.domain_path, self.problem_path, self.get_state, self.planner)
        executive.initialize(services)
        self.sock.send_one_message(INITILIZE_EXECUTIVE)

        while True:
            message = self.sock.recv_int()
            if message == DONE:
                self.report_card = pickle.loads(self.sock.recv_one_message())
                return self.report_card
            next_action = executive.next_action()

            services.on_action(next_action)
            if next_action is None:
                next_action = '(reach-goal)'
            self.sock.send_one_message(next_action)
