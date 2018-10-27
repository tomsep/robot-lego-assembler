from __future__ import division
from functools import partial

from legoassembler import script_build_tools
from legoassembler.communication import URServer, URClient
from legoassembler.build import *


IP_PC = '192.168.137.1'
IP_UR = '192.168.137.2'
PORT_UR = 30002  # 30002 for UR secondary client
PORT_PC = 30000
SCRIPTS_FOLDER = 'urscripts'
GRIPPER_BASE_DEF = 'urscripts/gripper_urcap_preamble.script'
GRIPPER_FUN_DEF = 'urscripts/gripper_function_definition.script'
Z_CLEARANCE = 0.3  # At which height the arm is moving across the board. Meters.
PLATFORM_CALIBR_PATH = '../platform_calibr.json'
CAMERA_CALIBR_PATH = '../camera_calibr.json'


def upload_script(script, gripper_base_def, gripper_fun_def, ip_ur, port_ur, ip_pc, port_pc):

    client = URClient()
    client.connect(ip_ur, port_ur)

    script = script_build_tools.build(script, gripper_base_def, gripper_fun_def, ip_pc, port_pc)

    client.send(script)

if __name__ == '__main__':

    # Setup
    # -------------------
    serv = URServer(IP_PC, PORT_PC)

    # Preload script building with all but the script name itself
    load_script = partial(upload_script, gripper_base_def=GRIPPER_BASE_DEF,
                gripper_fun_def=GRIPPER_FUN_DEF, ip_ur=IP_UR, port_ur=PORT_UR, ip_pc=IP_PC, port_pc=PORT_PC)


    # TODO: connect to cam server (or could be mv's job)
    # TODO: create mv
    mv = None

    # Environment calibration
    # -------------------
    if input('Load previous environment calibration data [Y/n]: ') == 'Y':
        with open(PLATFORM_CALIBR_PATH) as f:
            environment = json.load(f)
    else:
        script = '{}/calibrate_platform.script'.format(SCRIPTS_FOLDER)
        load_script(script=script)
        serv.accept()
        environment = paltform_calibration(serv, Z_CLEARANCE)
        with open(PLATFORM_CALIBR_PATH, 'w') as outfile:
            json.dump(environment, outfile)
        serv.close()

    # Build preview
    #-------------------
    if input('Live preview calibration poses [Y/n]: ') == 'Y':
        script = '{}/build_preview.script'.format(SCRIPTS_FOLDER)
        load_script(script)
        serv.accept()
        build_preview(serv, environment)
        serv.close()

    # Camera calibration
    # -------------------
    if input('Load previous camera calibration data [Y/n]: ') == 'Y':
        with open(CAMERA_CALIBR_PATH) as f:
            camera_calibr = json.load(f)
    else:
        script = '{}/calibrate_camera.script'.format(SCRIPTS_FOLDER)
        load_script(script=script)
        serv.accept()
        camera_calibr = camera_calibration(serv, mv)
        with open(CAMERA_CALIBR_PATH, 'w') as outfile:
            json.dump(camera_calibr, outfile)
        serv.close()

    # Building
    # -------------------
    if input('Start building [Y/n]: ') == 'Y':
        script = '{}/build.script'.format(SCRIPTS_FOLDER)
        load_script(script)
        serv.accept()
        build(serv, environment, camera_calibr, mv)
        serv.close()

