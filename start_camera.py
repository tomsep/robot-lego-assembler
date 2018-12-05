import struct
import socket
import yaml
import os
import sys

from legoassembler.camera_server import start

if __name__ == '__main__':

    filepath = os.path.dirname(sys.argv[0])
    filepath = os.path.abspath(filepath)

    confpath = os.path.join(filepath, 'config.yml')

    with open(confpath, 'r') as f:
        cfg = yaml.load(f)

    cfg_net = cfg['network']['raspi']

    # TODO: handle PiCameraValueError (when param out of valid range)

    while True:
        try:
            start(cfg_net['port'])
        except (socket.error, struct.error):
            print('Connection error. Service is restarted immediately.')