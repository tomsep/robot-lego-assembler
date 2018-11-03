# -*- coding: utf-8 -*-
from __future__ import division, print_function
import cv2 as cv
import yaml
from mock import MagicMock

from legoassembler.vision import *
from legoassembler.vision import _find_bricks_of_color


def _load_color_definitions():
    with open('../../config.yml', 'r') as f:
        cfg = yaml.load(f)
    return cfg['bricks']['colors']


def _find_bricks_of_color_when_multiple_matches():
    """ Test that correct amount of matches is returned"""

    fpath = './img/various_duplos_1.jpg'
    img = cv.imread(fpath, cv.IMREAD_COLOR)

    colors = _load_color_definitions()

    bricks = _find_bricks_of_color(img, colors['red'])

    assert len(bricks) == 2


def test_find_bricks_of_color():
    """ Test values returned for a brick"""

    fpath = './img/rect_x163_y123_-15-degree-from_xaxis_' \
            'color_hue27%_width200_length100.jpg'
    colors = {'testcol': ([46, 250, 250], [53, 255, 255])}  # Hue 27%
    img = cv.imread(fpath, cv.IMREAD_COLOR)
    center = (163, 123)  # x, y. Image origin at top left corner.
    width = 100  # pixels. Shorter is the width
    length = 200  # pixels
    angle = math.radians(-15)  # from x-axis
    correct = {'cx': center[0], 'cy': center[1], 'area': width * length,
                'angle': angle}

    res = _find_bricks_of_color(img, colors['testcol'])
    assert len(res) == 1

    eps_pix = 3
    for key in ['cx', 'cy']:
        assert abs(res[0][key] - correct[key]) < eps_pix

    assert abs(res[0]['dimensions'][0] - width) < eps_pix
    assert abs(res[0]['dimensions'][1] - length) < eps_pix

    assert abs(res[0]['area'] - correct['area']) < 0.03 * correct['area']

    eps_angle = math.radians(2)
    assert abs(res[0]['angle'] - angle) < eps_angle


class TestMachineVision:

    def test_calibration(self, monkeypatch):
        """ Test pixels_in_mm and TCP center point calculation using test image using
        custom 'red' color. """

        # Load test image
        fpath = './img/rect_x143_y294_30degree_color_hsv20%_side95px.jpg'
        colors = {'red': ([46, 250, 250], [53, 255, 255])}
        color = 'red'
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

        mv.calibrate(side_mm, color, draw=False)

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

        res = mv.find_brick('test', size=(1, 1))

        # From tcp to rect midp
        actual_dist_x = (rect_midp[0] - fake_tcp_xy[0]) / actual_pixels_in_mm
        actual_dist_y = (rect_midp[1] - fake_tcp_xy[1]) / actual_pixels_in_mm

        eps = 5
        assert abs(res['x'] - actual_dist_x) < eps
        assert abs(res['y'] - actual_dist_y) < eps

        # Both positive y quarters are ok
        assert abs(res['angle']) - actual_angle < math.radians(3)

