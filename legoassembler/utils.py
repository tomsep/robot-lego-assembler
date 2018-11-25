from __future__ import division
import numpy as np
from math import radians


def rectangle_angle_2d(points):
    """ Angle (-pi/2..pi/2) of the shortest edge w.r.t x-axis

    Parameters
    ----------
    points : array
        shape=(4, 2) i.e. 2D (x, y) points.

        Consecutive points must form edges orthogonal
        to neighboring ones, e.g.
        a --------- b
        |           |
        |           |
        d --------- c

    Returns
    -------
    float
        Angle -pi/2 to pi/2, w.r.t x-axis.

    Raises
    ------
    ValueError
        If points are not 2D, i.e. not arrays of length 2.

        If points do not form a rectangle or are in wrong order.

    """

    if type(points) != np.ndarray:
        points = np.array(points)

    if np.shape(points)[1] != 2:
        raise ValueError('Points are not 2D.')

    if not is_rectangle(points):
        raise ValueError('Points do not form a rectangle. Not all edges '
                         'are orthogonal or the order of points '
                         'is incorrect.')

    # Compute unit vectors for all edges
    edges = np.roll(points, -1, axis=0) - points
    norms = np.transpose(np.linalg.norm(edges, axis=1))[:, np.newaxis]

    min_edge = edges[np.argmin(norms)]
    angle = np.arctan2(min_edge[1], min_edge[0])
    return wrap_to_half_pi(angle)


def rectangle_center_2d(points):
    """ Center x,y of a rectangle

    Consecutive points must form edges orthogonal
    to neighboring ones, e.g.
    a --------- b
    |           |
    |           |
    d --------- c


    Parameters
    ----------
    points : array
        shape=(4, 2) i.e. 2D (x, y) points.

    Returns
    -------
    list[float, float]
        (x,y) center point.

    """

    if type(points) != np.ndarray:
        points = np.array(points)

    if np.shape(points)[1] != 2:
        raise ValueError('Points are not 2D.')

    if not is_rectangle(points):
        raise ValueError('Points do not form a rectangle. Not all edges '
                         'are orthogonal or the order of points '
                         'is incorrect.')

    # Using diagonal opposite points
    cx = (points[0, 0] + points[2, 0]) / 2
    cy = (points[0, 1] + points[2, 1]) / 2

    return [cx, cy]


def is_rectangle(points, eps=1e-6):
    """ Test if given 4 points form a rectangle

    Order of points matters. Consecutive points must form edges orthogonal
    to neighboring ones, e.g.
    a --------- b
    |           |
    |           |
    d --------- c

    Also rectangle must not have zero length edges.

    Parameters
    ----------
    points : array
        shape=(4, 2) or (4, 3) e.g. 2D or 3D points.
    eps : float
        Maximum allowed difference from ideal value.

    Returns
    -------
    bool
        If the points for a rectangle.

    """

    if type(points) != np.ndarray:
        points = np.array(points)

    if np.shape(points)[1] == 2:
        # Make vector 2D to 3D by adding 0
        points = np.append(points, np.zeros((4, 1)), axis=1)

    # Compute unit vectors for all edges
    edges = np.roll(points, -1, axis=0) - points
    norms = np.transpose(np.linalg.norm(edges, axis=1))[:, np.newaxis]

    # Check for 0 length edges
    if np.any(norms <= eps):
        return False

    unitvecs = edges / norms

    # Cross between all neighboring edges
    cross = np.cross(unitvecs, np.roll(unitvecs, 1, axis=0))

    # Check all cross products equal
    diff = np.abs(cross - cross[0])
    if np.all(diff <= np.array([0, 0, 0]) + eps):
        return True
    else:
        return False


def wrap_to_2pi(x):
    """ Wrap radian angle to between (-2pi .. 2pi) """
    pi = radians(180)
    if x > 2 * pi:
        return x - np.floor(x / (2 * pi)) * 2 * pi
    elif x < -2 * pi:
        return x + np.floor(x / (2 * pi)) * 2 * pi
    else:
        return x


def wrap_to_pi(x):
    """ Wrap radian angle to between (-pi .. pi) """
    x = wrap_to_2pi(x)
    pi = radians(180)
    if x > pi:
        return x - pi
    elif x < -pi:
        return x + pi
    else:
        return x


def wrap_to_half_pi(x):
    """ Wrap radian angle to between (-pi/2 .. pi/2) """
    x = wrap_to_pi(x)
    pi = radians(180)
    if x > pi / 2:
        return x - pi
    elif x < -pi / 2:
        return x + pi
    else:
        return x