from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np
import math
import json
import yaml
from copy import deepcopy


class MachineVision:

    def __init__(self, client, color_defs, cam_params, invert_x=False, invert_y=False):
        self.client = client
        self.cam_params = cam_params
        self.colors = color_defs

        self._invert_x = invert_x
        self._invert_y = invert_y

    def calibrate(self, side_mm, draw=True):
        """ Gathers camera calibration info from still picture of a square brick

        Captures an image and finds the largest brick of color 'red'.
        Assumes that the largest brick is square. Then
            * Computes pixel to mm conversion.
            * Finds TCP (Tool Center Point) in the image's coordinate system.

        Parameters
        ----------
        side_mm : float
            Width (mm) of the calibration brick (shape is square).
        draw : bool

        Raises
        ------
        NoMatches
            If not a single matching brick of color.

        """
        img = remote_capture(self.client, self.cam_params)
        color_code = 'red'
        bricks = find_bricks(img, self.colors[color_code], draw)
        if bricks == []:
            raise NoMatches('No bricks of color "{}" found.'.format(color_code))
        largest = max(bricks, key=lambda x: x['area'])  # brick with largest area

        side_as_pixels = math.sqrt(largest['area'])
        self.pixels_in_mm = side_as_pixels / side_mm

        self.tcp_xy = (largest['cx'], largest['cy'])

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

    def dist_to_largest_brick(self, color, draw=True):
        """ Get relative distance as mm to the largest brick of color

        Parameters
        ----------
        color
        draw

        Returns
        -------
        dict
                          x (mm) y (mm)       radians
            {'distance': (float, float), 'angle': float}

        Raises
        ------
        NoMatch
            If not a single block of color found.

        """

        img = remote_capture(self.client, self.cam_params)
        bricks = find_bricks(img, color, draw)

        brick = max(bricks, key=lambda x: x['area'])

        if brick == []:
            raise NoMatches

        midp_brick = (brick['cx'], brick['cy'])
        dist_mm = self._distance_from_p1(self.tcp_xy, midp_brick, as_mm=True)

        return {'distance': dist_mm, 'angle': brick['angle']}

    def _distance_from_p1(self, p1, p2, as_mm):

        dist_pix = [p2[0] - p1[0], p2[1] - p1[0]]

        if self._invert_x:
            dist_pix[0] = dist_pix[0] * -1

        if self._invert_y:
            dist_pix[1] = dist_pix[1] * -1

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


def find_bricks(img, color, draw=True):
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

    rects = _bounding_rectangles(img, contours, draw)

    return rects


def save_img(img, fname):
    # TODO: What if dir doesn't exist?
    cv.imwrite(fname, img)


def _find_largest_brick(img, color, draw=True):
    bricks = find_bricks(img, color, draw)
    if bricks == []:
        return None
    return max(bricks, key=lambda x: x['area'])  # brick with largest area


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
    color
    draw

    Returns
    -------

    """

    #blurred = cv.GaussianBlur(np.copy(img), (5, 5), 0)  # reduces noise
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Binary image by range
    mask = cv.inRange(hsv, np.array(color[0]), np.array(color[1]))

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

    # Calculate diagonal differences for x and y.
    width = abs(points[0][0] - points[2][0])
    height = abs(points[0][1] - points[2][1])

    return int(width * height)


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

        cx, cy = _rectangle_center(points)
        area = _rectangle_area(points)

        # Find longest edge
        edges = [(points[i], points[i + 1]) for i in range(len(points) - 1)]
        edges.append((points[0], points[-1]))

        # Rectangle angle
        edge_lengths = [abs(e[1][0] - e[0][0]) + abs(e[1][1] - e[0][1]) for e in edges]
        edge_longest = edges[np.argmax(edge_lengths)]
        # Get angle for longest edge
        angle = math.atan2(edge_longest[1][1] - edge_longest[0][1],
                           edge_longest[1][0] - edge_longest[0][0])

        rect = {'cx': cx, 'cy':cy, 'area': area, 'points': points, 'angle': angle}
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
