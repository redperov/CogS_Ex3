import struct
import pickle
import functools
import SocketServer
import errno

from pddlsim.simulator import Simulator
from pddlsim.fd_parser import FDParser
from remote_executive import RemoteExecutive
from socket_utils import *
from messages import *
from tmpdir import TmpDir


class SimulatorHandler(SocketServer.BaseRequestHandler):
    TMP_ROOT = '.tmp'

    def __init__(self, request, client_address, server, timeout, domain_path, problem_path):
        self.timeout = timeout
        self.domain_path = domain_path
        self.problem_path = problem_path
        self.provide_pddls = not domain_path is None
        self.tmp_dir = None
        SocketServer.BaseRequestHandler.__init__(
            self, request, client_address, server)

    def simulate(self, sim, problem_path, executor):
        def init(): return executor.initialize(None)
        sim.simulate()
        return sim.simulate_with_funcs(problem_path, init, executor.next_action)

    def handle(self):
        self.request.settimeout(self.timeout)
        try:
            sock = BufferedSocket(self.request)
            if self.provide_pddls:
                print 'Sending pddls'
                sock.send_one_message(SENDING_PDDLS)
                sock.send_file(self.domain_path)
                sock.send_file(self.problem_path)
            else:
                print 'Waiting for pddls'
                sock.send_one_message(WAITING_FOR_PDDLS)
                sock.get_file(self.domain_path)
                sock.get_file(self.problem_path)

            parser = FDParser(self.domain_path, self.problem_path)
            sim = Simulator(parser)
            remote = RemoteExecutive(sock, sim)
            remote.initialize(None)
            # sim.simulate_with_funcs(self.problem_path, lambda :
            # remote.initialize(None), remote.next_action)
            # self.simulate(sim, self.problem_path, remote)
            sim.simulate(remote.next_action)
            sock.send_int(DONE)
            sock.send_one_message(pickle.dumps(sim.report_card))
            print(
                'Reached goal!' if sim.report_card.success else 'Failed to reach goal')
        except socket.error, e:
            if e.errno == errno.ECONNRESET:
                # Handle disconnection -- close & reopen socket etc.
                print('disconnected')
            elif e.errno == errno.ETIMEDOUT:
                print('timedout')
            else:
                # Other error, re-raise
                raise

    def setup(self):
        if not self.provide_pddls:
            self.tmp_dir = TmpDir(self.TMP_ROOT)
            self.domain_path = self.tmp_dir.join('domain.pddl')
            self.problem_path = self.tmp_dir.join('problem.pddl')
        return SocketServer.BaseRequestHandler.setup(self)

    def finish(self):
        if self.tmp_dir:
            del self.tmp_dir
        return SocketServer.BaseRequestHandler.finish(self)


class InvalidSimulatorHandlerConfigsError(Exception):
    pass


class SimulatorForkedTCPServer(SocketServer.ForkingMixIn, SocketServer.TCPServer):

    def __init__(self, server_address):
        self.timeout = 60
        self.domain_path = None
        self.problem_path = None

        SocketServer.TCPServer.allow_reuse_address = True
        SocketServer.TCPServer.__init__(
            self, server_address, SimulatorHandler)

    @staticmethod
    def default():
        return SimulatorForkedTCPServer(("localhost", 9999))

    def finish_request(self, request, client_address):
        self.RequestHandlerClass(
            request, client_address, self, self.timeout, self.domain_path, self.problem_path)

    def provide_pddls(self, domain_path, problem_path):
        if (domain_path is None) or (problem_path is None):
            raise InvalidSimulatorHandlerConfigsError(
                "Please supply both a domain and a problem path")
        self.domain_path = domain_path
        self.problem_path = problem_path
        return self


def start(host="localhost", port=9999, requests_to_serve=-1):
    """
    Start a remote simulator server with host and port
    requests_to_serve will serve the specified amount
    default is -1 which will server forever
    """
    server = SimulatorForkedTCPServer((host, port),)

    print("Server starting...")
    if requests_to_serve == -1:
        # this will keep running until Ctrl-C
        server.serve_forever()
    else:
        for _ in xrange(requests_to_serve):
            server.handle_request()


if __name__ == "__main__":
    start()
