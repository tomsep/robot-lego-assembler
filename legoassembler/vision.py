from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np
import math
import json
import yaml
from copy import deepcopy

from legoassembler.utils import rectangle_angle_2d, rectangle_center_2d


class MachineVision:

    def __init__(self, client, color_defs, cam_params):
        self.client = client
        self.cam_params = cam_params
        self.colors = color_defs

    def calibrate(self, side_mm, color, draw=True):
        """ Gathers camera calibration info from a still picture of a square brick

        Captures an image and finds the best square dimension match brick of color 'red'.
            * Computes pixel to mm conversion.
            * Finds TCP (Tool Center Point) in the image's coordinate system.

        Parameters
        ----------
        side_mm : float
            Width (mm) of the calibration brick (shape is square).
        color : str
            Color name for the calibration block used.
        draw : bool

        Raises
        ------
        NoMatches
            If not a single matching brick of color within margin of error found.

        """

        margin = 0.1
        size = (1, 1)
        img = remote_capture(self.client, self.cam_params)
        bricks = _find_bricks_of_color(img, self.colors[color], draw)

        brick = _best_rect_ratio_match(bricks, size, margin)[0]

        target_ratio = size[0] / size[1]
        brick_ratio = brick['dimensions'][0] / brick['dimensions'][1]

        if abs(target_ratio - brick_ratio) > margin:
            raise NoMatches('Found {} bricks of color {} but none of them were within '
                            'margin of error {}%.'.format(len(bricks), color, margin*100))

        side_as_pixels = math.sqrt(brick['area'])
        self.pixels_in_mm = side_as_pixels / side_mm

        self.tcp_xy = (brick['cx'], brick['cy'])

    def save_calibration(self, fname):
        data = {'pixels_in_mm': self.pixels_in_mm,
                'tcp_xy': self.tcp_xy}
        with open(fname, 'w') as f:
            yaml.dump(data, f)

    def load_calibration(self, fname):
        with open(fname, 'r') as f:
            data = yaml.load(f)

        self.pixels_in_mm = data['pixels_in_mm']
        self.tcp_xy = data['tcp_xy']

    def find_brick(self, color, size, margin=0.2, draw=True):
        """ Localize brick with best dimension match and specified color


        Parameters
        ----------
        color : str
            Color name.
        size : array-like [float, float]
            Width and length dimensions of the brick to find.
        margin : float
            How much the match may differ at most. Ex. 0.2 == 20%.
        draw : bool
            If the visualizations should be drawn.

        Returns
        -------
        dict{'x': float, 'y': float, 'angle': float}
                (mm)        (mm)         radians
            Brick position relative to the TCP (tool center point). Units millimeters and
            radians.

            Coordinate system used:
            ^
            |
            y
            |
            0/0---x--->
            where angle is given w.r.t y-axis (-pi/4 .. pi/4] rad i.e. (-90 .. +90] deg.

        Raises
        ------
        NoMatches
            If no brick found within given margin or error.

        """

        img = remote_capture(self.client, self.cam_params)
        bricks = _find_bricks_of_color(img, self.colors[color], draw)

        try:
            bricks = _best_rect_ratio_match(bricks, size, margin)
        except ValueError:
            raise NoMatches


        for brick in bricks:

            brick['x_mm'], brick['y_mm'] = self._distance_from_p1(self.tcp_xy,
                                                                  (brick['cx'],
                                                                   brick['cy']),
                                                                  as_mm=True)
            brick['y_mm'] *= -1  # Change y axis direction
            brick['ratio'] = brick['dimensions'][0] / brick['dimensions'][1]

        target_ratio = size[0] / size[1]
        bricks = filter(lambda x: abs(target_ratio - x['ratio']) <= margin, bricks)


        if len(bricks) == 0:
            raise NoMatches

        bricks = sorted(bricks, key=lambda x: math.sqrt(x['x_mm'] ** 2 + x['y_mm'] ** 2))
        brick = bricks[0]

        match = {'x': brick['x_mm'], 'y': brick['y_mm'], 'angle': brick['angle']}
        return match

    def _distance_from_p1(self, p1, p2, as_mm):

        dist_pix = [p2[0] - p1[0], p2[1] - p1[1]]

        if as_mm:
            dist_mm = (dist_pix[0] / self.pixels_in_mm, dist_pix[1] / self.pixels_in_mm)
            return dist_mm
        else:
            return tuple(dist_pix)


class NoMatches(Exception):
    """ Error thrown when the system could not locate the element it was looking for.
    """
    pass


def remote_capture(client, cam_params):
    """ Remotely capture image

    Parameters
    ----------
    client
    cam_params

    Returns
    -------
    ???
    """
    # TODO: docstring

    client.send(json.dumps(cam_params))
    img_str = client.recv()

    # Format
    nparr = np.fromstring(img_str, np.uint8)
    return cv.imdecode(nparr, cv.IMREAD_COLOR)


def save_img(img, fname):
    # TODO: What if dir doesn't exist?
    cv.imwrite(fname, img)


def _find_bricks_of_color(img, color, draw=True):
    """ Find all bricks of certain color

    Parameters
    ----------
    img
    color
    draw

    Returns
    -------
    list[dict, ..]
        List of rectangles with keys 'cx', 'cy', 'area', 'points'.

    """
    mask = _mask_by_color(img, color, draw)

    if draw:
        draw_on = img
    else:
        draw_on = None
    contours = _find_contours(mask, draw_on)

    bricks = _bounding_rectangles(img, contours, draw)

    return bricks


def _img_midpoint(img):
    yx = np.shape(img)[:2]  # take only first 2 dims
    return [int(yx[1]), int(yx[0])]


def _distance_from_img_center(img, point):

    shape = np.shape(img)
    midpoint = [shape[1] / 2, shape[0] / 2]

    dx = abs(point[0] - midpoint[0])
    dy = abs(point[1] - midpoint[1])

    return math.sqrt(dx**2 + dy**2)


def _mask_by_color(img, color, draw=True):
    """ Create binary mask by color (HSV) range

    Values in range are white and other values black.

    Parameters
    ----------
    img
    color : dict
        Has ranges for HSV colors to mask by. If multiple keys using OR logical oper.

        Example
        {'1': [[0, 0, 0], [10, 10, 10]],
        '2': [[15, 0, 0], [16, 10, 10]]}
        so the mask includes Hues from 0-10 and 15-16 with SV of 0-10, 0-10.

    draw

    Returns
    -------

    """

    #blurred = cv.GaussianBlur(np.copy(img), (5, 5), 0)  # reduces noise
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Binary mask image by range
    mask = None
    for rang in color:
        if mask is None:
            mask = cv.inRange(hsv, np.array(rang[0]), np.array(rang[1]))
        else:
            mask_ = cv.inRange(hsv, np.array(rang[0]), np.array(rang[1]))
            mask = cv.bitwise_or(mask, mask_)

    kernel = np.ones((13, 13), np.uint8)
    morphed = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    if draw:
        cv.imshow('mask', morphed)
        cv.waitKey(1)

    return morphed


def _find_contours(mask, draw_on=None):
    """ Find countours from binary image

    Parameters
    ----------
    mask
    img
    draw

    Returns
    -------

    """
    _, contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    if draw_on is not None:
        img_ = np.copy(draw_on)
        draw_color = (0, 0, 0)
        cv.drawContours(img_, contours, -1, draw_color, thickness=2)
        cv.imshow('contours', img_)
        cv.waitKey(1)

    return contours


def _rectangle_area(points):
    """ Area of an rectangle
    Assumes points are in order.

    Returns
    -------
    int

    """

    lengths = _edge_lengths(points)
    area = lengths[0] * lengths[1]

    return area


def _bounding_rectangles(img, contours, draw=True):
    """ Fit minimum bounding rectangles to contours

    Parameters
    ----------
    img
    contours
    draw

    Returns
    -------

    """

    rects = []

    img_ = np.copy(img)

    for contour in contours:
        # Get bounding rectangle corner points
        minrect = cv.minAreaRect(contour)
        points = cv.boxPoints(minrect)

        try:
            cx, cy = rectangle_center_2d(points)
            cx, cy = int(cx), int(cy)
            angle = rectangle_angle_2d(points)
        except ValueError:
            # min rect has 0 len edges
            continue
        area = _rectangle_area(points)

        dims = _rectangle_dimensions(points)

        rect = {'cx': cx, 'cy': cy, 'area': area,
                'angle': angle, 'a': minrect[0], 'b': minrect[1], 'dimensions': dims}
        rects.append(rect)

        if draw:
            draw_color = (0, 0, 0)
            size = 2

            # Draw arrow
            length = 15
            x_arr = int(cx + length * math.cos(angle))
            y_arr = int(cy + length * math.sin(angle))
            cv.arrowedLine(img_, (cx, cy), (x_arr, y_arr), draw_color,size, tipLength=0.3)

            # Draw bounding rectangle
            edges = [(points[i], points[i + 1]) for i in range(len(points) - 1)]
            edges.append((points[0], points[-1]))
            for edge in edges:
                cv.line(img_, tuple(edge[0]), tuple(edge[1]), draw_color, size)

            # Draw center point
            cv.circle(img_, (cx, cy), size * 2, draw_color, -1)

            cv.imshow('bounding', img_)
            cv.waitKey(1)

    return rects


def _draw_rectangle(img, points):
    """ Draw rectangle to image
    """

    draw_color = (0, 0, 0)
    size = 2

    edges = [(points[i], points[i + 1]) for i in range(len(points) - 1)]
    edges.append((points[0], points[-1]))

    # Draw bounding rectangle
    for edge in edges:
        cv.line(img, tuple(edge[0]), tuple(edge[1]), draw_color, size)


def _annotate_rectangle(img, rect, text):
    """ Draw annotation text to rectangle
    """

    color = (66, 99, 22)
    origin = (rect['cx'], rect['cy'])
    cv.putText(img, text, origin, cv.FONT_HERSHEY_SIMPLEX,
               fontScale=0.6, color=color, thickness=1)


def _rectangle_center(points):

    # Using diagonal opposite points
    cx = (points[0][0] + points[2][0]) / 2
    cy = (points[0][1] + points[2][1]) / 2

    return int(cx), int(cy)


def _filter_by_area(bricks, area, margin):
    """ Get blocks that match area within margin %

    Parameters
    ----------
    bricks
    area
    margin : float
        Ex. 0.1 = 10% margin of error.

    Returns
    -------

    """

    filtered = []

    for brick in bricks:
        diff = abs(area - brick['area'])
        if diff / area < margin:
            filtered.append(deepcopy(brick))

    return filtered


def _edge_lengths(points):
    """ Compute edge lengths from ordered points
    Assumes points are in order.

    Returns
    -------
    list[float, ..]

    """

    if len(points) != 4:
        raise ValueError('Expected 4 points instead of {}'.format(len(points)))

    lengths = []

    for i in range(-2, 3):
        dx = abs(points[i][0] - points[i + 1][0])
        dy = abs(points[i][1] - points[i + 1][1])
        length = math.sqrt(dx ** 2 + dy ** 2)
        lengths.append(length)

    return lengths


def _rectangle_dimensions(points):
    """ Width and height of rectangle from 4 points

    Width is the shorter of the edges and length the longer one.

    Returns
    -------
    tuple(float, float)

    """
    ls = _edge_lengths(points)[:2]
    return (min(ls), max(ls))


def _best_rect_ratio_match(bricks, size, margin):

    def _ratio(_size):
        if _size[1] == 0:
            return 0
        else:
            return _size[0] / _size[1]

    target_ratio = _ratio(size)

    bricks = filter(lambda x: abs(_ratio(x['dimensions']) - target_ratio) < target_ratio * margin, bricks)
    return sorted(bricks, key=lambda x: abs(_ratio(x['dimensions']) - target_ratio))