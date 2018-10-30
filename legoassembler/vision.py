from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np
import math
import json
from copy import deepcopy


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
    """ Find bricks of certain color

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

    # bricks = filter_by_area(rects, area=2300, margin=0.7)
    #
    # # Draw bricks left after filtering
    # if draw:
    #     for brick in bricks:
    #         draw_rectangle(img, brick['points'])
    #         annotate_rectangle(img, brick, 'area={}'.format(brick['area']))
    #
    #     cv.imshow('found', img)
    #     cv.waitKey(1)

    # return bricks
    return rects


def save_img(img, fname):
    # TODO: What if dir doesn't exist?
    cv.imwrite(fname, img)


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

        rect = {'cx': cx, 'cy':cy, 'area': area, 'points': points}
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


def _get_unit_pixel_area(client, color, cam_params):
    """ Unit pixel area from a still picture of single 2x2 brick

    Unit pixel area is the are that one stud in the brick occupies, i.e.
    2x2 brick's unit area is 1/4th of the full area.

    Parameters
    ----------
    client : Client
    color : tuple(float, float, float)
        HSV color code.
    cam_params : dict
        Camera parameters.

    Returns
    -------
    int
        Unit pixel area.

    """

    img = remote_capture(client, cam_params)

    bricks = find_bricks(img, color, draw=True)

    if len(bricks) == 1:
        return int(bricks[0]['area'] / 4)
    else:
        raise ValueError('Expected to find only 1 brick but found {}'.format(len(bricks)))


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
