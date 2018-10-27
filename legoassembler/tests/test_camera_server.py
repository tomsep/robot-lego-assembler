# -*- coding: utf-8 -*-
from __future__ import division
from legoassembler.camera_server import start
from legoassembler.communication import Client

import json
import threading

IP = 'localhost'
PORT = 23455


def test_camera_server_comms(monkeypatch):
    """ Requests camera capture and shutdowns the camera server

    Mocks capture function and inserts a known return value. Also
    tests the camera parameters the function was provided.

    """

    params = {'iso': 300}
    capture_return = b'001'

    def mock_capture(camera, cam_params):
        assert cam_params['iso'] == 300
        return capture_return

    monkeypatch.setattr('legoassembler.camera_server.capture', mock_capture)

    t = threading.Thread(target=start, args=(IP, PORT))
    t.setDaemon(True)
    t.start()

    client = Client()
    client.connect(IP, PORT)
    client.send(json.dumps(params))

    data = client.recv()

    assert data == capture_return

    # Send termination message
    client.send(json.dumps(None))


def test_camera_server_with_real_capture():
    """ Requests an image and asserts the received data is PNG

    Shutdown message is sent to the server after receiving the data.
    """

    params = {'iso': 300, 'resolution': (1280, 720)}


    t = threading.Thread(target=start, args=(IP, PORT))
    t.setDaemon(True)
    t.start()

    client = Client()
    client.connect(IP, PORT)
    client.send(json.dumps(params))

    data = client.recv()

    assert data[:4] == '\x89PNG'  # Assert PNG file

    # Send termination message
    client.send(json.dumps(None))
    client.close()