import struct
import socket
import yaml

from legoassembler.camera_server import start

if __name__ == '__main__':

    with open('config.yml', 'r') as f:
        cfg = yaml.load(f)

    cfg_net = cfg['network']['raspi']

    # TODO: handle PiCameraValueError (when param out of valid range)

    while True:
        try:
            start(cfg_net['ip'], cfg_net['port'])
        except (socket.error, struct.error):
            print('Connection error. Service is restarted immediately.')