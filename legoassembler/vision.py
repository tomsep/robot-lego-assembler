from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np
import math
import json

HUE_RANGES = {'red': (np.array((0, 100, 100)), np.array((10, 255, 255))),
              'bright_yellow': (np.array((10, 200, 100)), np.array((35, 255, 255))),
              'green': (np.array((60, 200, 50)), np.array((85, 255, 200)))}


def contour_center(contour):
    mom = cv.moments(contour)
    cx = int(mom["m10"] / mom["m00"])
    cy = int(mom["m01"] / mom["m00"])
    return (cx, cy)


def detect_bricks_by_color(img, color_name, draw=True):
    """ Get one part center point, angle and edges

    Parameters
    ----------
    img
    color_name
    draw

    Returns
    -------
    int, int, float, list
        cx, cy, angle, edges

    """

    bricks = []

    img = cv.GaussianBlur(img, (5, 5), 0)  # reduces noise
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Binary image by range
    mask = cv.inRange(hsv, HUE_RANGES[color_name][0], HUE_RANGES[color_name][1])

    _, contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    for contour in contours:
        area = cv.contourArea(contour)

        # Center point of contour
        cx, cy = contour_center(contour)

        # Get bounding rectangle corner points
        minrect = cv.minAreaRect(contour)
        points = cv.boxPoints(minrect)

        # Find longest edge
        edges = [(points[i], points[i+1]) for i in range(len(points) - 1)]
        edges.append((points[0], points[-1]))
        edge_lengths = [abs(e[1][0] - e[0][0]) + abs(e[1][1] - e[0][1]) for e in edges]
        edge_longest = edges[np.argmax(edge_lengths)]

        # Get angle for longest edge
        angle = math.atan2(edge_longest[1][1]-edge_longest[0][1] , edge_longest[1][0]-edge_longest[0][0])

        specs = {'cx': cx, 'cy': cy, 'angle': angle, 'area': area}
        bricks.append(specs)

        # Draw and annotate
        # -----------------
        # Draw orientation arrow

        color = (0, 0, 0)  # draw color
        size = 2

        if draw:
            length = 20
            x_arr = int(cx + length * math.cos(angle))
            y_arr = int(cy + length * math.sin(angle))
            cv.line(img, (x_arr, y_arr), (cx, cy), color, size)

            # Draw bounding rectangle
            for edge in edges:
                cv.line(img, tuple(edge[0]), tuple(edge[1]), color, size)

            # Draw center point
            cv.circle(img, (cx, cy), size*2, color, -1)

        if draw:
            cv.imshow('', img)
            cv.waitKey(1)

    return bricks


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


def get_unit_pixel_area(client, color, cam_params):
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

    bricks = detect_bricks_by_color(img, color, draw=True)

    if len(bricks) == 1:
        return int(bricks[0]['area'] / 4)
    else:
        raise ValueError('Expected to find only 1 brick but found {}'.format(len(bricks)))


def find_brick(client, size, color, cam_params, unit_pixel_area):

    img = remote_capture(client, cam_params)

    bricks = detect_bricks_by_color(img, color, draw=True)

    # Find bricks of correct size by an error margin
    margin = 0.1  # 0..1, e.g. 0.1 = 10%

    target_pixel_area = size[0] * size[1] * unit_pixel_area
    for brick in bricks:
        diff = abs(brick['area'] - target_pixel_area)
        if diff / target_pixel_area < margin:
            return brick  # found match
    else:
        return None  # No matches

    # Distance from center
    #img_center = (int(img.shape[1]/2), int(img.shape[0]/2))
    #dist = abs(img_center[1] - cy) + abs(img_center[0] - cx)
