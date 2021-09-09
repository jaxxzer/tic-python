#!/usr/bin/env python3

# PingProxy.py
# Connect multiple udp clients to a single serial device

from brping import PingParser
import serial
import socket
import time
from collections import deque
import errno


class TicProxy(object):
    def __init__(self, device=None, port=None, debug=False):
        self.debug = debug

        ## A serial object for ping device comms
        self.device = device

        ## UDP port number for server
        self.port = port

        if self.device is None:
            raise Exception("A device is required")

        if self.port is None:
            raise Exception("A port is required")

        ## Connected client dictionary
        self.client = None

        ## Socket to serve on
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setblocking(False)
        self.socket.bind(('0.0.0.0', self.port))

    ## Run proxy tasks
    # @return None
    def run(self):
        try:
            data, self.client = self.socket.recvfrom(4096)
            if (self.debug):
                print(">", self.client, data.hex())
            self.device.write(data)
        except Exception as e:
            if e.errno == errno.EAGAIN:
                pass  # waiting for data
            else:
                print("Error reading data", e)

        # read serial device
        device_data = self.device.read(self.device.in_waiting)

        # send serial data to client
        if device_data and self.client != None:  # don't write empty data
            if self.debug:
                print("<", self.client, device_data.hex())
            self.socket.sendto(device_data, self.client)

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Ping udp proxy server.")
    parser.add_argument('--device', action="store", required=True, type=str, help="Ping device serial port.")
    parser.add_argument('--port', action="store", type=int, default=9090, help="Server udp port.")
    parser.add_argument('--debug', action="store_true", help="enable debug output")
    args = parser.parse_args()

    s = serial.Serial(args.device, 9600, exclusive=True)
    proxy = TicProxy(s, args.port, args.debug)

    while True:
        proxy.run()
        time.sleep(0.001)
