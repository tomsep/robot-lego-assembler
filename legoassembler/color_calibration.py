from __future__ import division, print_function
import cv2 as cv
import numpy as np
from functools import partial
import scipy.optimize
from sys import maxsize, version_info
import yaml
import os

# Python2 'raw_input' is equal to python3 'input'
if version_info[0] == 2:
    input = raw_input


def _select_quadrilateral(img, win_name=''):
    """ Gather 4 points based of user input and draw the result

    Parameters
    ----------
    img : numpy.ndarray
    win_name : str
        Name of the imshow window.

    Returns
    -------
    list
        List of 4 points [[int, int], ...]

    """

    clone = img.copy()

    points = []

    def _select_point(event, x, y, flags, param):
        # Save and draw points and when 4 draw lines
        if event == cv.EVENT_LBUTTONUP:
            if not len(points) == 4:
                points.append([x, y])
                cv.circle(clone, (x, y), 2, (0, 0, 0), thickness=4)
                cv.imshow(win_name, clone)
                if len(points) == 4:
                    print('Press any key (inside the image window) to continue.')

            if len(points) == 4:
                for i in range(-2, 2):
                    cv.line(clone, tuple(points[i]), tuple(points[i + 1]), color=(0, 0, 0))
                cv.imshow(win_name, clone)

    cv.imshow(win_name, clone)
    cv.setMouseCallback(win_name, _select_point)

    cv.imshow(win_name, img)
    cv.waitKey(0)
    return points


def _bound_width(bounds, array, min_prop):
    """Return bound width if the bounds encapsulate at least 'min_prop'
    proportion of the total sum of 'array'
    """

    start, end = bounds[0], bounds[1]

    if not 0 <= start < end:
        return maxsize

    total_area = np.sum(array)
    area = np.sum(array.take(range(start, end), mode='wrap'))

    if area / total_area >= min_prop:
        return (end - start)
    else:
        return maxsize


def _hsv_range_in_selection(img, points, min_prop):
    """ Optimize mimimum ranges for channels of img with constraint that a
    range must contain atleast 'min_prop' proportion of the total value counts.

    Only area within 'points' is considered.

    Each channel may have one or two ranges.

    Parameters
    ----------
    img : numpy.ndarray
    points : list[[int, int], ...]
        List of points.
    min_prop : float

    Returns
    -------
    numpy.ndarray
        Ranges for each channel.
        E.g. [[[0, 5], [175,180]],  # Hue. Two ranges: 0->5 and 175->180
             [[120, 150]],            # Saturation. Just one range
             [[50, 75]]]              # Value.


    """

    channel_ranges = []

    # Mask values not within contours
    mask = np.ones_like(img, dtype=np.uint8) * 255  # Black image
    points = np.array(points, dtype=np.int32)
    mask = cv.fillPoly(mask, points, (0, 0, 0))  # fill contour with white

    # Select unmasked values (values inside contour)
    selection_img = np.ma.masked_array(img, mask)

    # Channels: hue (0-180), saturation (0-255) and value (0-255)
    for i, max_val in [(0, 180), (1, 255), (2, 255)]:
        # Brute forces minimum bounds in two steps. First coarse and the fine
        freqs, _ = np.histogram(np.ma.compressed(selection_img[:, :, i]),
                               bins=max_val, range=(0, max_val))

        # Funct to optimize: minimize bound width
        fun = partial(_bound_width, array=freqs, min_prop=min_prop)

        # Get coarse estimate
        step = 10
        ranges = (slice(0, max_val*2, step),) * 2
        opt = scipy.optimize.brute(fun, ranges, disp=True, finish=None)
        opt = [int(x) for x in opt]

        # Using previous estimate make a better one
        ranges = (slice(opt[0] - step, opt[0] + step, 1),
                  slice(opt[1] - step, opt[1] + step, 1))
        opt = scipy.optimize.brute(fun, ranges, disp=True, finish=None)
        opt = [int(x) for x in opt]

        # split into two ranges if opt bounds over max value
        if opt[1] > max_val:
            if opt[0] > max_val:
                opt[0] -= max_val
            opt = [[opt[0], max_val], [0, opt[1] - max_val]]
        else:
            opt = [opt]

        channel_ranges.append(opt)

    return channel_ranges


def _format_ranges(ranges):
    """ Create a list of ranges. Range: [h1, s1, v1] to [h2, s2, v2]

    Examples
    --------
    input: [
            [[0, 5], [175, 180]],   # hue, two ranges
            [100, 200],             # saturation, only one range
            [50, 75]                # value, only one range
            ]

    Because Hue has two ranges then the result has two ranges.
    Satur. and val. have only one range and so their first (and now only one) is
    duplicated to each result range.

    result: [
        [[0, 100, 50], [5, 200, 75]],     # first range
        [[175, 100, 50], [180, 200, 75]]  # second
    ]

    """

    # Make all len 2 if any one is len 2
    if max([len(x) for x in ranges]) == 2:
        hues = ranges[0]
        if len(hues) == 1:
            hues *= 2

        saturs = ranges[1]
        if len(saturs) == 1:
            saturs *= 2

        vals = ranges[2]
        if len(vals) == 1:
            vals *= 2
    else:
        hues, saturs, vals = ranges[0], ranges[1], ranges[2]

    new_ranges = []
    for i in range(len(hues)):
        start = [hues[i][0], saturs[i][0], vals[i][0]]
        end = [hues[i][1], saturs[i][1], vals[i][1]]
        new_ranges.append([start, end])

    return new_ranges


def _widen_hsv_color_range(color_range, proportion):
    """Widens range by proportion
    new lower bound = old - (old * proportion / 2)
    new upper bound = old + (old * proportion / 2)
    """
    new_range = [[None, None, None], [None, None, None]]
    max_values = [180, 255, 255] # h, s ,v
    for i in range(3):
        width = color_range[1][i] - color_range[0][i]
        low = int(color_range[0][i] - (width * proportion / 2))
        if low < 0:
            low = 0
        high = int(color_range[1][i] + (width * proportion / 2))
        if high > max_values[i]:
            high = max_values[i]

        new_range[0][i] = low
        new_range[1][i] = high

    return new_range


def color_calibration_from_img(img, widen_prop=0.35, min_prop=0.98, fname='color_definitions.yml'):
    """ Do user assisted color calibration

    User is instructed to select 4 points within the image. These points specify an area.
    The area is used to define a color as ranges for HSV values. Values are added
    to a file. Old entries of the same color name are overwritten.

    The HSV ranges selection basis:
        * minimum range width
        * range contains atleast 'min_prop' proportion of the total count of values.

    Color definitions are saved in yaml format.
    E.g. red might have entry:
    red:
        - - [177, 126, 41]
          - [180, 246, 122]
        - - [0, 126, 41]
          - [3, 246, 122]
    which means that red is defined as two ranges:
        hue, satur, val
        [177, 126, 41] to [180, 246, 122]
        and
        [0, 126, 41] to [3, 246, 122]

    Parameters
    ----------
    img : numpy.ndarray
    widen_prop : float
        Proportion (0..1) to widen the optimized color range. I.e. if hue has range
        10 to 25 after optimization the 10 is lowered according to widen_prop
        and similarly 25 is made larger.
    min_prop : float
        Proportion (0..1).
    fname : str
        Filename for the color definitions.


    """

    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    win_name = ''

    while True:
        print('Point & click 4 points to specify an area used to define a color.')
        points = _select_quadrilateral(img, win_name)
        color_def = _hsv_range_in_selection(img_hsv, [points], min_prop)
        name = input('Give color name: ')

        color_defs = {}
        # Read and then write (replacing old definition)
        if os.path.isfile(fname):
            with open(fname, 'r') as f:
                color_defs = yaml.safe_load(f)
                if color_defs is None:  # if empty file
                    color_defs = {}

        color_def = _format_ranges(color_def)

        # Widen range by widen_prop bit
        for i in range(len(color_def)):
            color_def[i] = _widen_hsv_color_range(color_def[i], widen_prop)
        color_defs[name] = color_def

        with open(fname, 'w') as f:
            f.write(yaml.dump(color_defs))
        print('Color definition added for "{}".'.format(name))

        if input('Do another? [Y/n]: ') != 'Y':
            break

    print('Calibration ended.')


if __name__ == '__main__':
    img_name = 'img.jpg'
    img = cv.imread(img_name, cv.IMREAD_COLOR)
    color_calibration_from_img(img)
