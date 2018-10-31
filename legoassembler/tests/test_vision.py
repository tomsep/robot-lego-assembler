# -*- coding: utf-8 -*-
from __future__ import division
import cv2 as cv
import yaml
from mock import MagicMock

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


class TestMachineVision:

    def test_calibration(self, monkeypatch):
        """ Test pixels_in_mm and TCP center point calculation using test image using
        custom 'red' color.

        """

        # Load test image
        fpath = './img/rect_x143_y294_30degree_color_hsv20%_side95px.jpg'
        colors = {'red': ([46, 250, 250], [53, 255, 255])}
        rect_actual_width_pixels = 95
        side_mm = 10
        actual_pixels_in_mm = rect_actual_width_pixels / side_mm
        rect_actual_midpoint = (143, 294)

        img = cv.imread(fpath, cv.IMREAD_COLOR)

        def _mock_remote_capture(*args, **kwargs):
            return img

        monkeypatch.setattr('legoassembler.vision.remote_capture', _mock_remote_capture)

        client = MagicMock()
        mv = MachineVision(client, colors, {})

        mv.calibrate(side_mm, draw=False)

        eps = 5
        # Test midpoint
        assert abs(mv.tcp_xy[0] - rect_actual_midpoint[0]) < eps
        assert abs(mv.tcp_xy[1] - rect_actual_midpoint[1]) < eps

        # Test pixels_in_mm
        assert abs(mv.pixels_in_mm - actual_pixels_in_mm) < eps

    def test_find_largest_brick(self, monkeypatch):
        """ Test that the found brick has correct center point and angle

        Both +y quarters are ok for the angle.

        """

        # Load test image
        fpath = './img/rect_x143_y294_30degree_color_hsv20%_side95px.jpg'
        colors = {'test': ([46, 250, 250], [53, 255, 255])}
        rect_actual_width_pixels = 95
        side_mm = 10
        actual_pixels_in_mm = rect_actual_width_pixels / side_mm
        rect_midp = (143, 294)
        actual_angle = math.radians(30)

        img = cv.imread(fpath, cv.IMREAD_COLOR)

        def _mock_remote_capture(*args, **kwargs):
            return img

        monkeypatch.setattr('legoassembler.vision.remote_capture', _mock_remote_capture)

        client = MagicMock()
        mv = MachineVision(client, colors, {})

        fake_tcp_xy = (50, 25)

        # Add calibration
        mv.pixels_in_mm = actual_pixels_in_mm
        mv.tcp_xy = fake_tcp_xy

        res = mv.dist_to_largest_brick(colors['test'])

        # From tcp to rect midp
        actual_dist_x = (rect_midp[0] - fake_tcp_xy[0]) / actual_pixels_in_mm
        actual_dist_y = (rect_midp[1] - fake_tcp_xy[1]) / actual_pixels_in_mm

        eps = 5
        assert abs(res['distance'][0] - actual_dist_x) < eps
        assert abs(res['distance'][1] - actual_dist_y) < eps

        # Both positive y quarters are ok
        assert abs(res['angle']) - actual_angle < eps

