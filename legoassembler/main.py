from __future__ import division
from __future__ import print_function
import yaml
import os
import socket
from warnings import warn

from legoassembler import script_build_tools
from legoassembler.communication import URServer, URClient, Client
import legoassembler.build
from legoassembler.robot import Robot


def run(cfg):
    """ Run main app
    """
    host, cam_client, ur_client = _start_networking(cfg)    # Server and clients
    mv = _mv_setup(cfg, cam_client)                               # Machine vision

    rob = Robot(cfg['network']['ur']['ip'])

    if input('Teach platform? [Y/n]: ') == 'Y':
        load = False
    else:
        load = True
    platform_calib = _teach_platform(cfg, rob, load)

    if input('Preview taught platform? [Y/n]: ') == 'Y':
        _preview_taught_platform(cfg, rob, platform_calib)

    if input('Calibrate camera? [Y/n]: ') == 'Y':
        load = False
    else:
        load = True
    camera_calib = _calibrate_camera(cfg, host, ur_client, mv, load)

    if input('Test camera calibration? [Y/n]: ') == 'Y':
        _test_camera_calibration(cfg, host, ur_client, cam_client,
                                 platform_calib, camera_calib)

    if input('Start building? [Y/n]: ') == 'Y':
        raise NotImplementedError('Building not impl.')


def _teach_platform(cfg, rob, load):

    if load:
        with open(cfg['calibration_data']['platform'], 'r') as f:
            calib = yaml.load(f)
    else:
        with open(cfg['calibration_data']['platform'], 'w') as f:
            #_upload_scipt(cfg, 'teach_platform', ur_client)
            #host.accept()
            calib = legoassembler.build.teach_platform(rob)
            f.write(yaml.dump(calib))
            #host.close()
    return calib


def _calibrate_camera(cfg, host, ur_client, mv, load):

    if load:
        with open(cfg['calibration_data']['camera'], 'r') as f:
            calib = yaml.load(f)
    else:
        with open(cfg['calibration_data']['camera'], 'w') as f:
            _upload_scipt(cfg, 'calibrate_camera', ur_client)
            host.accept()
            travel_height = cfg['environment']['travel_height']
            calib = legoassembler.build.calibrate_camera(host, mv, travel_height)
            f.write(yaml.dump(calib))
            host.close()
    return calib


def _calibrate(cfg, host, ur_client, mv, load):
    """ Start calibration OR load previous calibration

    Also calibrates 'mv'.

    Parameters
    ----------
    cfg : dict
    host : URServer
    ur_client : URClient
    mv : MV
    load : bool
        To load or not to load previous calibration from file.

    """

    with open(cfg['calibration_data'], 'w+') as f:
        if load:
            calib = yaml.load(f)
        else:
            _upload_scipt(cfg, 'calibrate', ur_client)
            host.accept()
            calib = legoassembler.build.\
                    calibrate(host, cfg['environment']['travel_height'], mv)

            f.write(yaml.dump(calib))
            host.close()

        mv.calibrate(calib['unit_pixel_area'])


def _build(cfg, host, ur_client, mv):

    _upload_scipt(cfg, 'build', ur_client)
    host.accept()
    legoassembler.build.build(host, cfg['environment']['travel_height'], None, mv)
    host.close()


def _preview_taught_platform(cfg, rob, calib):

    #_upload_scipt(cfg, 'preview_taught_platform', ur_client)
    #host.accept()
    legoassembler.build.preview_taught_platform(rob, calib, cfg['environment']['travel_height'])
    #host.close()


def _test_camera_calibration(cfg, host, ur_client, cam_client, calib_platf, calib_cam):

    _upload_scipt(cfg, 'test_camera_calibration', ur_client)
    travel_h = cfg['environment']['travel_height']
    colors = cfg['bricks']['colors']
    host.accept()
    legoassembler.build.test_camera(host, cam_client, calib_platf, calib_cam, travel_h, colors)
    host.close()


def _preview(cfg, host, ur_client, mv):

    _upload_scipt(cfg, 'preview_taught_platform', ur_client)
    host.accept()
    legoassembler.build.build_preview(host, cfg['environment']['travel_height'])
    host.close()


def _load_config():
    """ Load configuration from config.yml

    Returns
    -------
    dict

    """

    with open('config.yml', 'r') as f:
        cfg = yaml.load(f)
    _discover_scripts(cfg)
    return cfg


def _discover_scripts(config, pckg_name='legoassembler'):
    """ Construct and validate absolute paths to scripts

    Parameters
    ----------
    config : dict
        Configuration loaded from config.yml
    pckg_name : str
        Name of the package in that contains the source code.

    """

    root = os.path.dirname(os.path.abspath(__file__))
    scripts = config['ur_scripts']
    folder = pckg_name + os.sep + scripts['directory']

    for key, script in scripts.items():
        if key != 'directory':
            new_path = os.path.join(root, folder, script)
            if not os.path.isfile(new_path):
                raise ValueError('File "{}" does not exist'.format(new_path))
            scripts[key] = new_path


def _start_networking(cfg):
    """ Start host server and connect clients

    Allows program to continue even if some of the connections fail to open.
    Warnings are thrown.

    Parameters
    ----------
    cfg : dict

    Returns
    -------
    URServer, Client, URClient
        Host, camera client, ur client.

        Host: serve data to UR controller.
        Camera client: request data from camera pc (raspi)
        UR client: send data to UR controller

    """

    net = cfg['network']

    # SERVER to server UR
    ip = net['host']['ip']
    port = net['host']['port']
    try:
        host = URServer(ip, port)
    except socket.error:
        host = None
        warn('Could not create host for {}:{}. Communication with the robot '
              'will not work.'.format(ip, port))

    # Client to access camera (raspi)
    ip = net['raspi']['ip']
    port = net['raspi']['port']
    cam_client = Client()
    try:
        cam_client.connect(net['raspi']['ip'], net['raspi']['port'])
    except socket.error:
        warn('Could not connect to {}:{}. Communication with the camera '
                      'will not work.'.format(ip, port))

    # Client to send scripts to UR
    ip = net['ur']['ip']
    port = net['ur']['port']
    ur_client = URClient()
    try:
        ur_client.connect(ip, port)
    except socket.error:
        warn('Could not connect to {}:{}. Communication with the robot '
              'will not work.'.format(ip, port))

    return host, cam_client, ur_client


def _upload_scipt(cfg, name, ur_client):
    """ Build and upload script to UR controller

    Parameters
    ----------
    cfg : dict
    name : str
        Name of the script (key in 'cfg')
    ur_client : URClient
        Client for communicating data to UR controller.

    """

    scripts = cfg['ur_scripts']
    path = scripts[name]

    script = script_build_tools.build(path,
                                      scripts['gripper_urcap_preamble'],
                                      scripts['gripper_function_definition'],
                                      cfg['network']['host']['ip'],
                                      cfg['network']['host']['port'])

    ur_client.send(script)


def _mv_setup(cfg, cam_client):

    mv = None

    return mv


def _load_calib(cfg):
    """ Load calibration YAML

    Parameters
    ----------
    cfg : dict

    Returns
    -------
    dict

    """

    # TODO: suggest doing calibration if file missing

    with open(cfg['calibration_data'], 'r') as f:
        return yaml.load(f)

