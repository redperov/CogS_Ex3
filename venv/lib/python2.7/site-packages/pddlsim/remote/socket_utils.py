import socket, sys, os
import struct


# SENDING_DOMAIN, SENDING_PROBLEM, REQUEST_PLAN, SENDING_PLAN = "send_domain","send_problem", "request_plan","send_plan"

BUFFER_SIZE = 1024

class BufferedSocket():
    def __init__(self, sock):
        self.sock = sock
        self.buf = b''

    def recvall(self, count):
        buf = b''
        while count:
            if self.buf:
                newbuf = self.buf
            else:
                newbuf = self.sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        if count < 0:
            self.buf = newbuf[count:]
        return buf

    def send_one_message(self, data):
        length = len(data)
        self.send_int(length)
        self.sock.sendall(data)

    def recv_one_message(self):
        length = self.recv_int()
        return self.recvall(length)

    def recv_int(self):
        numbuf = self.recvall(4)
        if numbuf is None: return None
        num, = struct.unpack('!I', numbuf)
        return num

    def send_int(self, num):
        self.sock.sendall(struct.pack('!I', num))

    def send_file(self, filename):
        size = os.path.getsize(filename)
        with open(filename,'rb') as f:
            self.send_one_message(f.read())

    def recv_file(self, size, save_to_path):
        remaining_size = size        
        file_content = self.recvall(remaining_size)
        with open(save_to_path, 'wb') as f:
            f.write(file_content)
        # with open(save_to_path, 'wb') as f:
        #     print 'file opened'
        #     while remaining_size > 0:
        #         print('receiving data...')
        #         data = self.sock.recv(BUFFER_SIZE)
        #         print('data=%s', (data))
        #         if not data:
        #             break
        #         remaining_size -= len(data)
        #         # write data to a file
        #         f.write(data)

    def get_file(self, save_to_path):
        # size = self.recv_int()
        # self.recv_file(size,save_to_path)
        file_content = self.recv_one_message()
        with open(save_to_path, 'wb') as f:
            f.write(file_content)
