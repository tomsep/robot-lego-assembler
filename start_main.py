from __future__ import division
from functools import partial
import yaml
import os

from legoassembler import script_build_tools
from legoassembler.communication import URServer, URClient
from legoassembler.build import *


def load_config():
    """ Load configuration from config.yml

    Returns
    -------
    dict

    """

    with open('config.yml', 'r') as f:
        cfg = yaml.load(f)
    discover_scripts(cfg)
    return cfg


def discover_scripts(config, pckg_name='legoassembler'):
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


def upload_script(script, gripper_base_def, gripper_fun_def, ip_ur, port_ur, ip_pc, port_pc):

    client = URClient()
    client.connect(ip_ur, port_ur)

    script = script_build_tools.build(script, gripper_base_def, gripper_fun_def, ip_pc, port_pc)

    client.send(script)


if __name__ == '__main__':

    cfg = load_config()
    cfg_net = cfg['network']
    cfg_scripts = cfg['ur_scripts']
    cfg_calib = cfg['calibration_data']
    cfg_env = cfg['environment']

    # Setup
    # -------------------

    serv = URServer(cfg_net['host']['ip'], cfg_net['host']['port'])

    # Preload script building with all but the script name itself
    load_script = partial(upload_script, gripper_base_def=cfg_scripts['gripper_urcap_preamble'],
                gripper_fun_def=cfg_scripts['gripper_function_definition'],
                          ip_ur=cfg_net['ur']['ip'], port_ur=cfg_net['ur']['port'],
                          ip_pc=cfg_net['host']['ip'], port_pc=cfg_net['host']['port'])


    # TODO: connect to cam server (or could be mv's job)
    # TODO: create mv
    mv = None

    # Environment calibration
    # -------------------
    if input('Load previous environment calibration data [Y/n]: ') == 'Y':
        with open(cfg_calib['platform']) as f:
            environment = json.load(f)
    else:
        load_script(script=cfg_scripts['calibrate_platform'])
        serv.accept()
        environment = paltform_calibration(serv, cfg_env['travel_height'])
        with open(cfg_calib['platform'], 'w') as outfile:
            json.dump(environment, outfile)
        serv.close()

    # Build preview
    #-------------------
    if input('Live preview calibration poses [Y/n]: ') == 'Y':
        load_script(cfg_scripts['build_preview'])
        serv.accept()
        build_preview(serv, environment)
        serv.close()

    # Camera calibration
    # -------------------
    if input('Load previous camera calibration data [Y/n]: ') == 'Y':
        with open(cfg_calib['camera']) as f:
            camera_calibr = json.load(f)
    else:
        load_script(cfg_scripts['calibrate_camera'])
        serv.accept()
        camera_calibr = camera_calibration(serv, mv)
        with open(cfg_calib['camera'], 'w') as outfile:
            json.dump(camera_calibr, outfile)
        serv.close()

    # Building
    # -------------------
    if input('Start building [Y/n]: ') == 'Y':
        load_script(cfg_scripts['build'])
        serv.accept()
        build(serv, environment, camera_calibr, mv)
        serv.close()

