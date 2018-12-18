""" Starts user assisted color calibration procedure. """

from __future__ import division, print_function
import sys

from legoassembler.communication import Client
from legoassembler.color_calibration import color_calibration_from_img
from legoassembler.vision import remote_capture
from legoassembler.utils import load_config

# Python2 'raw_input' is equal to python3 'input'
if sys.version_info[0] == 2:
    input = raw_input


if __name__ == '__main__':
    """ Capture image and for that do user-assisted color calibration
    """

    cfg = load_config()
    cfg_net = cfg['network']['raspi']
    fname = cfg['calibration_data']['colors']

    client = Client()
    client.connect(cfg_net['ip'], cfg_net['port'])

    cam_params = cfg['camera_parameters']
    print('Camera parameters used: {}'.format(cam_params))
    img = remote_capture(client, cam_params)

    color_calibration_from_img(img, widen_prop=0.4, min_prop=0.98, fname=fname)
