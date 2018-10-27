from legoassembler.camera_server import start
import argparse

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', required=True, help='ip address of this machine')
    ap.add_argument('--port', required=True, help='port to use')
    args = vars(ap.parse_args())

    start(args['ip'], int(args['port']))