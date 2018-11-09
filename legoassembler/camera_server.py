# -*- coding: utf-8 -*-
from __future__ import division
import json
import time
import io

from legoassembler.communication import Server


def start(ip, port):
    """ Start a server for serving images

    Server is shut when None message is received.

    Receives camera parameters as dictionary.

    See apply_cam_params function for more info about camera parameters.

    """

    # Imported here so that this module can be imported without picamera
    from picamera import PiCamera

    serv = Server(ip, port)
    serv.accept()
    # TODO: handle PiCameraValueError (when param out of valid range)
    with PiCamera() as camera:
        time.sleep(2)  # Allow camera to power up

        while True:
            cam_params = json.loads(serv.recv().decode('utf-8'))

            if cam_params is None:
                break

            image = capture(camera, cam_params)
            serv.send(image)

        serv.close()


def capture(camera, cam_params, iformat='jpeg'):
    """ Capture image

    Parameters
    ----------
    camera : PiCamera
    cam_params : dict
    iformat : str

    Returns
    -------
    bytes
        Image as bytes.

    """

    apply_cam_params(camera, cam_params)

    stream = io.BytesIO()
    camera.capture(stream, iformat, use_video_port=True)
    stream.seek(0)
    return stream.read()


def apply_cam_params(camera, params):
    """ Apply all changes to camera parameters

    Keys of 'params' dict are used as the attribute names, e.g.
    {'iso': 300} is applied as ´´camera.iso = 300´´.

    Only parameters that have changed from last time are applied.

    Parameters
    ----------
    camera : PiCamera
    params : dict

    """

    for key, new_val in params.items():

        old_val = getattr(camera, key)

        if new_val != old_val:
            setattr(camera, key, new_val)