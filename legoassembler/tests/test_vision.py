# -*- coding: utf-8 -*-
from __future__ import division
import cv2 as cv
import yaml

from legoassembler.vision import get_unit_pixel_area, find_brick


def color_definitions():
    with open('../../config.yml', 'r') as f:
        cfg = yaml.load(f)
    return cfg['bricks']['colors']


def test_finding_brick(monkeypatch):
    """ Test finding brick when unit pixel area is first calculated

    """

    def patch_capture(fpath):
        def mock_capture(*args, **kwargs):
            img = cv.imread(fpath, cv.IMREAD_COLOR)
            img = cv.resize(img, None, fx=0.2, fy=0.2, interpolation=cv.INTER_CUBIC)
            return img
        return mock_capture

    path = './bin/2x2_red_duplo.jpg'
    monkeypatch.setattr('legoassembler.vision.remote_capture', patch_capture(path))

    color_defs = color_definitions()
    color = color_defs['red']

    unit_px_area = get_unit_pixel_area(None, color, {})

    assert unit_px_area > 0

    path = './bin/various_2x4_duplos.jpg'
    monkeypatch.setattr('legoassembler.vision.remote_capture', patch_capture(path))

    size = (2, 4)
    bricks = find_brick(None, size, color, {}, unit_px_area)
    assert bricks is not None