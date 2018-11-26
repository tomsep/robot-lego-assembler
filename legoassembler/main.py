from __future__ import division
from __future__ import print_function
import yaml
import socket
from warnings import warn
import os
import sys

from legoassembler.communication import Client
import legoassembler.build
from legoassembler.robot import Robot
from legoassembler.vision import MachineVision
from legoassembler.lego import load_file, coordinates, number_of_bricks, printer


# Python2 'raw_input' is equal to python3 'input'
if sys.version_info[0] == 2:
    input = raw_input


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

    if input('Start building? [Y/n]: ') == 'Y':

        if input('Continue previous build? [Y/n]: ') == 'Y':
            load = True
        else:
            load = False
        _build(cfg, rob, mv, platform_calib, load)


def _teach_platform(cfg, rob, load):

    if load:
        with open(cfg['calibration_data']['platform'], 'r') as f:
            calib = yaml.load(f)
    else:
        with open(cfg['calibration_data']['platform'], 'w') as f:
            gripper = cfg['gripper']
            tcp = cfg['tcp']
            calib = legoassembler.build.teach_platform(rob, gripper, tcp)
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
            brick_2x2_length = cfg['brick_2x2_length']
            color = cfg['calibration_color']
            gripper = cfg['gripper']
            tcp = cfg['tcp']
            legoassembler.build.calibrate_camera(rob, mv, gripper, tcp, travel_height,
                                                 platf_calib, brick_2x2_length, color)
            mv.save_calibration(cfg['calibration_data']['camera'])


def _preview_taught_platform(cfg, rob, calib):
    gripper = cfg['gripper']
    tcp = cfg['tcp']
    legoassembler.build.\
        preview_taught_platform(rob, tcp, calib,
                                cfg['environment']['travel_height'],
                                gripper)


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

    # load color definitions
    with open(cfg['calibration_data']['colors'], 'r') as f:
        color_defs = yaml.safe_load(f.read())

    mv = MachineVision(cam_client, color_defs, {'iso': 800, 'resolution': [800, 600]})
    fpath = cfg['calibration_data']['camera']
    if os.path.isfile(fpath):
        mv.load_calibration(cfg['calibration_data']['camera'])
    return mv


def _load_build_plan(fname):
    lego_file = load_file(fname)
    plans = coordinates(lego_file)
    legos = number_of_bricks(plans)
    printer(legos, plans)

    return plans


def _build(cfg, rob, mv, platf_calib, load_state):

    fname = cfg['lego_model_path']
    plan = _load_build_plan(fname)
    travel_h = cfg['environment']['travel_height']
    unit_brick_dims = {'side': cfg['brick_2x2_length'] / 1000,
                       'base_height': cfg['brick_base_height'] / 1000}
    gripper = cfg['gripper']
    tcp = cfg['tcp']

    legoassembler.build.build(rob, mv, gripper, tcp, platf_calib, plan, travel_h,
                              unit_brick_dims, load_state)
