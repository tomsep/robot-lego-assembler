# -*- coding: utf-8 -*-
from __future__ import division
import cv2 as cv
import yaml

from legoassembler.vision import *


def _load_color_definitions():
    with open('../../config.yml', 'r') as f:
        cfg = yaml.load(f)
    return cfg['bricks']['colors']


def test_find_multiple_bricks_of_same_color():

    fpath = './img/various_duplos_1.jpg'
    img = cv.imread(fpath, cv.IMREAD_COLOR)

    colors = _load_color_definitions()

    bricks = find_bricks(img, colors['red'])

    assert len(bricks) == 2
