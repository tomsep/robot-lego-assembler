from __future__ import division, print_function
import yaml
import cv2 as cv
import time
import os


from legoassembler.vision import remote_capture
from legoassembler.communication import Client
from legoassembler.utils import load_config


def load_colors(fname):
    with open(fname, 'r') as f:
        return yaml.safe_load(f.read())

if __name__ == '__main__':

    cfg = load_config()
    rpi_network = cfg['network']['raspi']

    fname = 'color_definitions.yml'
    colors = load_colors(fname)
    client = Client()

    client.connect(rpi_network['ip'], rpi_network['port'])

    cam_params = cfg['camera_parameters']

    folder = './imageset/unprocessed/'
    while True:
        img = remote_capture(client, cam_params)
        cv.imshow('preview', img)
        cv.waitKey(20)
        time.sleep(0.1)
        if input('Save? [Y/n]: ') == 'Y':
            fname = folder + str(int(time.time())) + '.png'
            print(fname)
            cv.imwrite(fname, img)