from __future__ import division
import pytest
from math import radians
import numpy as np
from numpy.testing import assert_array_almost_equal

from legoassembler.utils import *


def test_rectangle_angle_2d():

    # rect x (width) = 2, y (length) = 4, shortest edge angle 0 deg from x-axis
    # Midpoint (x=1, y=2)
    rect_0deg = [[0, 0], [2, 0], [2, 4], [0, 4]]

    rect_90deg = [[0, 0], [4, 0], [4, 2], [0, 2]]

    res = rectangle_angle_2d(points=rect_0deg)
    assert_array_almost_equal(abs(res), radians(0))

    res = rectangle_angle_2d(points=rect_90deg)
    assert_array_almost_equal(res, radians(90))

    # Flip rect_0deg and assure it still is 0 deg and not -180
    res = rectangle_angle_2d(points=np.flip(rect_0deg, axis=0))
    assert_array_almost_equal(res, radians(0))

    # Test with wrong order points
    square_diagonal_points = [[0, 0], [2, 2], [2, 0], [0, 2]]
    with pytest.raises(ValueError):
        rectangle_angle_2d(square_diagonal_points)


def test_is_rectangle():
    """ Test 3D and 2D points"""

    # Rectangle, 3D points
    points3d = np.array([[301.7405, 514.9248, 1],
                         [274.68436, 402.83487, 2],
                         [378.26974, 377.83148, 2],
                         [405.3259, 489.92142, 1]])

    # Rectangle, 2D points
    points2d = points3d[:, :2]

    assert is_rectangle(points3d)
    assert is_rectangle(points2d)

    # Make not rectangle
    points3d_bad = points3d.copy()
    points3d_bad[3, 2] += 1

    assert not is_rectangle(points3d_bad)


def test_rectangle_center_2d():

    # rect x (width) = 2, y (length) = 4
    # Midpoint (x=1, y=2)
    rect_90deg = [[0, 0], [2, 0], [2, 4], [0, 4]]

    res = rectangle_center_2d(points=rect_90deg)
    assert type(res) == list
    assert_array_almost_equal(abs(np.array(res)), [1, 2])

