from legoassembler.camera_server import start
import yaml

if __name__ == '__main__':

    with open('config.yml', 'r') as f:
        cfg = yaml.load(f)

    cfg_net = cfg['network']['raspi']

    start(cfg_net['ip'], cfg_net['port'])