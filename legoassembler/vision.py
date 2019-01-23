from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np
import math
import json
import yaml
from copy import deepcopy
import tensorflow as tf
tf.enable_eager_execution()

from legoassembler.utils import rectangle_angle_2d, rectangle_center_2d


class MachineVision:

    def __init__(self, client, color_defs, cam_params):
        self.client = client
        self.cam_params = cam_params
        self.colors = color_defs
        self._is_calibrated = False

        model_path = './tfmodels/sobel-unet-color-psize/model.h5'
        self._model = tf.keras.models.load_model(model_path, compile=False)

    def calibrate(self, side_mm, color, draw=True):
        """ Gathers camera calibration info from a still picture of a square brick

        Captures an image and finds the best square dimension match brick of 'color'.
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

        self._is_calibrated = True

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

        self._is_calibrated = True

    def find_brick(self, color, size, margin=0.2, use_max_edge=False, draw=True, use_model=True):
        """ Localize brick with best dimension match and specified color


        Parameters
        ----------
        color : str
            Color name.
        size : array-like [float, float]
            Width and length dimensions of the brick to find.
        margin : float
            How much the match may differ at most before being invalid. Ex. 0.2 == 20%.
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

        if not self._is_calibrated:
            raise ValueError('Camera has not been calibrated or '
                             'no calibration has been loaded.')

        img = remote_capture(self.client, self.cam_params)

        if use_model:
            if color:
                target_color = self.colors[color]
            else:
                target_color = None
            bricks = _find_bricks_using_model(img, self._model, use_max_edge, (160, 160), (400, 400),
                                              target_color, draw)

        else:
            bricks = _find_bricks_of_color(img, self.colors[color], use_max_edge, draw)

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
        bricks = list(bricks)

        if len(bricks) == 0:
            raise NoMatches

        bricks = sorted(bricks, key=lambda x: math.sqrt(x['x_mm'] ** 2 + x['y_mm'] ** 2))
        brick = bricks[0]

        match = {'x': brick['x_mm'], 'y': brick['y_mm'], 'angle': brick['angle']}
        return match

    def color_in_region(self, color, region, min_area=0.3, draw=True):
        """ Match color within region of interest

        Also draws (unless disabled) ROI and text whether the match was found.

        Parameters
        ----------
        color : str
            Name of the color.
        region : list[[float, float], [float, float]]
            Region of interest (ROI) as pixel ranges of height and width.
        min_area : float
            Portion 0..1
        draw : bool

        Returns
        -------
        bool
            True if matched color area within ROI is larger than 'min_area' portion of the ROI.
            Else False.

        """

        # Region as Height range, width range, e.g. [[0, 150], [310, 500]]
        img = remote_capture(self.client, self.cam_params)

        masked = _mask_by_color(img, self.colors[color], no_morph=True, hue_only=True)
        cropped = masked[region[0][0]:region[0][1], region[1][0]:region[1][1]]

        # Check that cropped area has certain portion of white pixels
        if np.count_nonzero(cropped == 255) / np.size(cropped) >= min_area:
            found = True
        else:
            found = False

        if draw:
            draw_color = (0, 180, 0)
            cv.rectangle(img, (region[1][1], region[0][1]), (region[1][0], region[0][0]), draw_color, thickness=5)

            if found:
                text = 'match'
            else:
                text = 'no match'

            cv.putText(img, text, (region[1][1], region[0][1]), cv.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), thickness=6)
            cv.putText(img, text, (region[1][1], region[0][1]), cv.FONT_HERSHEY_PLAIN, 2, draw_color, thickness=3)

            cv.imshow('bounding', img)
            cv.waitKey(1)

        return found

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
    """ Remotely capture color image

    Parameters
    ----------
    client
    cam_params : dict
        Parameters for the camera. Parameter name must match the attribute name
        that PiCamera defines. See PiCamera documentation https://picamera.readthedocs.io

        For example to set resolution and iso values the dict would look like
        {'resolution': (800, 600), 'iso': 500}

    Returns
    -------
    numpy.ndarray
        Color image as an numpy array.

    """

    client.send(json.dumps(cam_params))
    img_str = client.recv()

    # Format
    nparr = np.fromstring(img_str, np.uint8)
    return cv.imdecode(nparr, cv.IMREAD_COLOR)


def save_img(img, fname):
    # TODO: What if dir doesn't exist?
    cv.imwrite(fname, img)


def _find_bricks_of_color(img, color, use_max_edge, draw=True):
    """ Find all objects of certain color

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

    bricks = _bounding_rectangles(img, contours, use_max_edge, draw)

    return bricks


def _find_bricks_using_model(img, model, use_max_edge, input_size, window_size, color=None, draw=True):
    """ Find bricks based on model
    Color ignorant if the 'model' is color as well.

    Parameters
    ----------
    model : tensorflow Keras model
    color : color HSV range to find
        If None colors are ignored and only shape is considered.

    """

    mask = img[:, :, 0].copy() * 0

    # Center crop ROI of 'window_size'
    shape = np.shape(mask)
    h_cent, w_cent = (shape[0] // 2, shape[1] // 2)
    mask_roi = mask[h_cent - window_size[0] // 2:h_cent + window_size[0] // 2,
               w_cent - window_size[1] // 2:w_cent + window_size[1] // 2]

    # Get segmentation prediction
    segmentation = _net_predict(img, model, input_size, window_size)

    if draw:
        cv.imshow('neural_network_output', segmentation)

    # Threshold red channel to binary image
    red_chnl = segmentation.copy()[:, :, 2]
    _, mask_roi[:, :] = cv.threshold(red_chnl, 255//2, 255, cv.THRESH_BINARY)

    if draw:
        draw_on = img
    else:
        draw_on = None

    # Find contours and fit boxes to mask
    contours = _find_contours(mask, draw_on)

    if color:
        contours = _filter_contours_by_color(img, contours, color)

    bricks = _bounding_rectangles(mask, contours, use_max_edge, draw)
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


def _mask_by_color(img, color, draw=True, no_morph=False, hue_only=False):
    """ Create binary mask by color (HSV) range

    Values in range are white and other values black.

    Parameters
    ----------
    img
    color : list
        Has ranges for HSV colors to mask by. If multiple ranges then using OR logical oper.

        Example
        [[[0, 0, 0], [10, 10, 10]], [[15, 0, 0], [16, 10, 10]]]
        so the mask includes Hues from 0-10 and 15-16 with SV of 0-10, 0-10.

    draw
    no_morph : bool
        To use closing operation or not.
    hue_only : bool
        If matching is done only using hue value or all HSV values.

    Returns
    -------

    """

    #blurred = cv.GaussianBlur(np.copy(img), (5, 5), 0)  # reduces noise
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Binary mask image by range
    mask = None
    for rang in color:
        if mask is None:
            if hue_only:
                rang = [(rang[0][0], 0, 0), (rang[1][0], 255, 255)]
            mask = cv.inRange(hsv, np.array(rang[0]), np.array(rang[1]))
        else:
            mask_ = cv.inRange(hsv, np.array(rang[0]), np.array(rang[1]))
            mask = cv.bitwise_or(mask, mask_)

    if no_morph:
        morphed = mask
    else:
        kernel = np.ones((13, 13), np.uint8)
        morphed = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    if draw:
        cv.imshow('mask', morphed)
        cv.waitKey(1)

    return morphed


def _find_contours(mask, draw_on=None):
    """ Find countours from binary image

    Not compatible with OpenCV 2 without small modification.

    Returns
    -------
    list

    """

    # CV2 would return only two values, not 3.
    _, contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    if draw_on is not None:
        img_ = np.copy(draw_on)
        draw_color = (200, 200, 0)
        cv.drawContours(img_, contours, -1, draw_color, thickness=2)
        cv.imshow('contours', img_)
        cv.waitKey(1)

    return contours


def _rectangle_area(points):
    """ Area of an rectangle
    Assumes points are in sequential order.

    Returns
    -------
    int

    """

    lengths = _edge_lengths(points)
    area = lengths[0] * lengths[1]

    return area


def _bounding_rectangles(img, contours, use_max_edge=False, draw=True):
    """ Fit minimum bounding rectangles to contours
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
            angle = rectangle_angle_2d(points, use_max_edge)
        except ValueError:
            # min rect has 0 len edges
            continue
        area = _rectangle_area(points)

        dims = _rectangle_dimensions(points)

        rect = {'cx': cx, 'cy': cy, 'area': area,
                'angle': angle, 'a': minrect[0], 'b': minrect[1], 'dimensions': dims}
        rects.append(rect)

        if draw:
            draw_color = (200, 200, 0)
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
    area : float
    margin : float
        Ex. 0.1 = 10% margin of error.

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
    """ Sort and filter list of bricks based on target ration and margin

    'margin' determines how easily a brick passes filtering. Results are sorted
    from best matching to worst before returning the list.

    """

    def _ratio(_size):
        if _size[1] == 0:
            return 0
        else:
            return _size[0] / _size[1]

    target_ratio = _ratio(size)

    bricks = filter(lambda x: abs(_ratio(x['dimensions']) - target_ratio) < target_ratio * margin, bricks)
    return sorted(bricks, key=lambda x: abs(_ratio(x['dimensions']) - target_ratio))


def _net_predict(img, model, input_size, window_size=(400, 400)):
    shape = np.shape(img)
    h_cent, w_cent = (shape[0] // 2, shape[1] // 2)
    roi = img[h_cent - window_size[0]//2:h_cent + window_size[0]//2, w_cent - window_size[1]//2:w_cent + window_size[1]//2, :]
    img_in = _parse_image_for_prediction(roi)

    pred = model.predict(np.expand_dims(img_in, axis=0), batch_size=1)
    pred = (pred * 255).astype(np.uint8)
    pred = cv.resize(pred[0], window_size)

    return pred


def _parse_image_for_prediction(image):
    """ Process png image
    """
    image = tf.convert_to_tensor(image, tf.uint8)
    def _process(_image, reverse_chans=False):


        # Don't use tf.image.decode_image, or the output shape will be undefined
        if reverse_chans:
            _image = tf.reverse(_image, axis=[-1])
        # This will convert to float values in [0, 1]
        _image = tf.image.convert_image_dtype(_image, tf.float32)

        _image = tf.image.resize_images(_image, [160, 160])
        return _image

    image = _process(image, reverse_chans=False)

    with tf.device('/cpu:0'):
        sobel = tf.image.sobel_edges(tf.expand_dims(image, axis=0))
        sobel = tf.squeeze(sobel)
        sobel = tf.reshape(sobel, [160, 160, 6])
        ass_shape = tf.debugging.assert_equal(tf.shape(sobel), tf.constant([160, 160, 6]))
        with tf.control_dependencies([ass_shape]):
            image = tf.concat([image, sobel], axis=2)

    return image


def _filter_contours_by_color(img, contours, color, min_prop=0.5, draw=True):

    filtered_conts = []

    for cont in contours:
        mask = np.zeros(np.shape(img), np.uint8)
        mask = cv.fillPoly(mask, [cont], (255, 255, 255))

        color_roi = np.zeros_like(img)
        np.putmask(color_roi, mask>0, values=img)

        masked_color = _mask_by_color(color_roi, color, draw)

        if np.count_nonzero(masked_color>0) / np.count_nonzero(mask[:, :, 0]>0) > min_prop:
            filtered_conts.append(cont)

    return np.array(filtered_conts)

