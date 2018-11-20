from __future__ import division
from __future__ import print_function
import yaml
import socket
from warnings import warn
import os

from legoassembler.communication import Client
import legoassembler.build
from legoassembler.robot import Robot
from legoassembler.vision import MachineVision
from legoassembler.lego import load_file, coordinates, number_of_bricks, printer

def run(cfg):
    """ Run main app
    """
    cam_client = _connect_to_camera_client(cfg)
    mv = _mv_setup(cfg, cam_client) # Machine vision

    cfg_net = cfg['network']

    if cfg['environment']['simulated']:
        print('Running in simulation mode')
        grip_def = None
    else:
        grip_def = cfg['grip_def_script']

    rob = Robot(cfg_net['ur']['ip'], cfg_net['host']['ip'],
                grip_def, cfg_net['host']['port'])

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
    _calibrate_camera(cfg, rob, mv, platform_calib, load)

    if input('Test camera? [Y/n]: ') == 'Y':
        _test_camera(cfg, rob, mv, platform_calib)

    if input('Start building? [Y/n]: ') == 'Y':
        _build(cfg, rob, mv, platform_calib)

    if input('Pickup Demo? [Y/n]: ') == 'Y':
        _pickup_demo(cfg, rob, mv, platform_calib)

def _teach_platform(cfg, rob, load):

    if load:
        with open(cfg['calibration_data']['platform'], 'r') as f:
            calib = yaml.load(f)
    else:
        with open(cfg['calibration_data']['platform'], 'w') as f:
            calib = legoassembler.build.teach_platform(rob)
            f.write(yaml.dump(calib))
    return calib


def _calibrate_camera(cfg, rob, mv, platf_calib, load):

    if load:
        with open(cfg['calibration_data']['camera'], 'r') as f:
            calib = yaml.load(f)
    else:
        with open(cfg['calibration_data']['camera'], 'w') as f:
            #_upload_scipt(cfg, 'calibrate_camera', ur_client)
            #host.accept()
            travel_height = cfg['environment']['travel_height']
            legoassembler.build.calibrate_camera(rob, mv, travel_height, platf_calib, 32, 'blue')
            mv.save_calibration(cfg['calibration_data']['camera'])

def _preview_taught_platform(cfg, rob, calib):
    legoassembler.build.preview_taught_platform(rob, calib, cfg['environment']['travel_height'])


def _connect_to_camera_client(cfg):
    """ Connect and return camera client

    Allows program to continue even if connection fails to open. In such case
    a warning is printed.

    Parameters
    ----------
    cfg : dict

    Returns
    -------
    Client

    """

    net = cfg['network']

    ip = net['raspi']['ip']
    port = net['raspi']['port']
    cam_client = Client()
    try:
        cam_client.connect(net['raspi']['ip'], net['raspi']['port'])
    except socket.timeout:
        warn('Timeout error: Could not connect to {}:{}. Communication with the camera '
             'will not work.'.format(ip, port))

    return cam_client


def _mv_setup(cfg, cam_client):

    mv = MachineVision(cam_client, cfg['bricks']['colors'], {'iso': 800, 'resolution': [800, 600]})
    fpath = cfg['calibration_data']['camera']
    if os.path.isfile(fpath):
        mv.load_calibration(cfg['calibration_data']['camera'])
    return mv

def _test_camera(cfg, rob, mv, platf_calib):
    legoassembler.build.test_camera(rob, mv, cfg['environment']['travel_height'],
                                    platf_calib, 'red')


def _load_build_plan(fname):
    lego_file = load_file(fname)
    plans = coordinates(lego_file)
    legos = number_of_bricks(plans)

    # make into millimeters
    mmfact = 32
    mmfact_h = 19.2
    for plan in plans:
        plan[:3] = [(plan[0] - 1) * mmfact_h, plan[1] * mmfact, plan[2] * mmfact]

    printer(legos, plans)

    return plans

def _build(cfg, rob, mv, platf_calib):

    fname = cfg['lego_model_path']
    plan = _load_build_plan(fname)
    travel_h = cfg['environment']['travel_height']
    legoassembler.build.build(rob, mv, platf_calib, plan, travel_h)

def _pickup_demo(cfg, rob, mv, platf_calib):
    legoassembler.build.pickup_demo(rob, mv, cfg['environment']['travel_height'],
                                    platf_calib, ['red', 'blue', 'yellow'])

