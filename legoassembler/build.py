"""

Assumptions:
        1. Base of the robot is parallel to the platform.
        2. Camera is mounted on the -y side of the robot wrist.
        3. Camera points to same direction as the gripper.
        4. Build and part areas are parallel and have approx. same height (z).

"""

from __future__ import division
from __future__ import print_function
import json
import time

from vision import *


def teach_platform(rob):

    build_area = []

    rob.popup('Hit continue to start guided calibration procedure.', blocking=True)
    rob.popup('Continue to initialize gripper.', blocking=True)

    rob.grip(closed=76)

    # TODO: teach safe starting position

    msg = 'Place 2x2 block on one corner of the build platform.' \
              ' Guide the arm to grab the block.'
    rob.teachmode(True, msg)

    #rob.popup('Place 2x2 block on one corner of the build platform.'
    #          ' Guide the arm to grab the block.', blocking=True)
    build_area.append(rob.get_tcp())

    msg = 'Place 2x2 block on the diagonal opposite corner of the build platform.' \
              ' Guide the arm to grab the block.'
    rob.teachmode(True, msg)
    #rob.popup('Place 2x2 block on the diagonal opposite corner of the build platform.'
    #          ' Guide the arm to grab the block.', blocking=True)
    build_area.append(rob.get_tcp())

    msg = 'Move the gripper to middle (ground level)' \
              ' of the pickup platform.'
    rob.teachmode(True, msg)
    #rob.popup('Move the gripper to middle (ground level)'
    #          ' of the pickup platform.', blocking=True)
    part_area = rob.get_tcp()

    rob.teachmode(False)
    rob.popup('Teaching finished!')

    env = {'taught_poses': {'build': build_area, 'part': part_area}}

    print('Teaching finished!')

    return env


def preview_taught_platform(rob, calib, travel_height):

    wait = 0.1
    vel = 0.3
    a = 0.1
    poses = calib['taught_poses']
    rob.popup('Press continue to start preview', blocking=True)

    pose = rob.get_tcp()
    pose[2] = poses['build'][0][2] + travel_height
    rob.move('l', pose, v=vel, a=a)

    # Open gripper
    rob.grip(closed=76)

    pose = deepcopy(poses['build'][0])
    pose[2] += travel_height
    rob.move('j', pose, v=vel, a=a)
    time.sleep(wait)
    rob.move('l', poses['build'][0], v=vel, a=a)
    time.sleep(wait)
    rob.move('l', pose, v=vel, a=a)
    time.sleep(wait)

    pose = deepcopy(poses['build'][1])
    pose[2] += travel_height
    rob.move('j', pose, v=vel, a=a)
    time.sleep(wait)
    rob.move('l', poses['build'][1], v=vel, a=a)
    time.sleep(wait)
    rob.move('l', pose, v=vel, a=a)
    time.sleep(wait)

    pose = deepcopy(poses['part'])
    pose[2] += travel_height
    rob.move('j', pose, v=vel, a=a)
    time.sleep(wait)
    rob.move('l', poses['part'], v=vel, a=a)
    time.sleep(wait)
    rob.move('l', pose, v=vel, a=a)
    time.sleep(wait)

    rob.popup('Preview finished!')
    print('Preview finished!')


def calibrate(server, travel_height, mv):
    """ Runs calibration sequence to create new calibration dictionary

    1. With teach mode enabled record three poses:
        a. Instruct user to move the TCP to one corner of the build platform.
        b. Same for diagonal opposite corner.
        c. Same for the mid point of the part area.

    2. Move to imaging position
        a. Instruct user to place 2x2 RED brick directly under the camera.
        b. Capture image and calculate and record unit pixel area.
        c. Send confirmation to end ur program.

    Parameters
    ----------
    server : URServer

    z_clearance : float
        How much higher than the platform should the arm (TCP) travel.
        Is used to calculate travel_z for the Environment, i.e.
        travel_z = max_of_z_from_poses + z_clearance

    Returns
    -------
    dict

    """

    build_area = []
    part_area = []

    def _append_xyz(arr, pose):
        p = (pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
        arr.append(p)

    # Build area
    for _ in range(2):
        pose = json.loads(server.recv()[1:])  # [:1] to to get [1, ..] from p[1, ..]
        print('Received build area pose: {}'.format(pose))
        _append_xyz(build_area, pose)

    # Part area
    pose = json.loads(server.recv()[1:])
    print('Received part area pose: {}'.format(pose))
    _append_xyz(part_area, pose)

    # Move to imaging pose
    server.recv(header='pose')
    imaging_pose = _parallel_to_floor(pose)
    _add_to_pose(imaging_pose, 'z', travel_height)
    server.send(imaging_pose)

    # Capture and calculate
    server.recv(header='capture')
    unit_area = mv.infer_unit_px_area()

    env = {'poses': {'build': build_area, 'part': part_area[0]},
           'unit_pixel_area': unit_area}

    return env


def test_camera(server, cam_client, calib_platf, calib_cam, travel_height, colors):

    #mv = MachineVision(colors)
    cam_params = {}
    img = remote_capture(cam_client, cam_params)

    while True:
        contours(img, colors['red'])


def calibrate_camera(rob, mv, travel_height, calib, brick2x2_side_mm, color):

    wait = 0.1
    vel = 3
    a = 0.1

    test_pose = calib['taught_poses']['build'][0]

    rob.popup('Press continue to start camera calibration', blocking=True)

    # Goto starting height
    pose = rob.get_tcp()
    pose[2] = test_pose[2] + travel_height
    rob.move('l', pose, v=vel, a=a)
    time.sleep(wait)

    # Open gripper
    rob.grip(closed=76)

    pose = deepcopy(test_pose)
    pose[2] += travel_height
    rob.move('j', pose, v=vel, a=a)
    time.sleep(wait)
    rob.move('l', test_pose, v=vel, a=a)
    time.sleep(wait * 2)
    rob.move('l', pose, v=vel, a=a)

    # Take calibration image
    mv.calibrate(brick2x2_side_mm, color)

    # Move aside
    pose = [-0.03, 0.02, 0, 0, 0, 0]
    rob.move('j', pose, v=vel, a=a, relative=True)
    time.sleep(wait)
    joints = [0, 0, 0, 0, 0, math.radians(0)]
    rob.move_joints('j', joints, v=vel, a=a, relative=True)
    time.sleep(wait)

    # Take new image
    match = mv.find_brick(color, (1, 1), margin=0.2, draw=True)
    #match = {'x': 30, 'y': -20, 'angle': math.radians(-15)}
    print(match)
    pose = [-match['y'] / 1000, match['x'] / 1000, 0, 0, 0, 0]
    rob.move('j', pose, v=vel, a=a, relative=True)
    time.sleep(wait)
    joints = [0, 0, 0, 0, 0, match['angle']]
    rob.move_joints('j', joints, v=vel, a=a, relative=True)
    time.sleep(wait)

    # Touch the match
    pose = rob.get_tcp()
    pose[2] = test_pose[2]
    rob.move('l', pose, v=vel, a=a)
    time.sleep(wait * 2)
    pose[2] = travel_height
    rob.move('l', pose, v=vel, a=a)
    time.sleep(wait)

    rob.popup('Camera calibration finished!')
    print('Camera calibration finished!')


def build_preview(server, env, z_clear=0.05):
    """ Run through (as preview) calibrated platform points with height (z) clearance

    Parameters
    ----------
    server : URServer
    env : dict
    z_clear : float
        Offset from the calibrated poses' z value. Meters.


    """

    def _add_z_clearance(pose, z):
        p = [pose[0],pose[1],pose[2] + z,pose[3],pose[4],pose[5]]
        return p

    params = {'wait_time': 0.3, 'velocity': 0.2, 'travel_z': env['travel_z']}
    communicate_parameters(server, params)

    for i in range(2):
        server.recv(header='pose')
        p = _add_z_clearance(env['build_area'][i], z_clear)
        server.send(str(p))

    for i in range(2):
        server.recv(header='pose')
        p = _add_z_clearance(env['part_area'][i], z_clear)
        server.send(str(p))


def build(server, env, instrc, mv):
    """ Runs build sequence

    Parameters
    ----------
    server : URServer
    env : dict
    instrc : ???
        Instructions for building
    mv : ???
        Machine vision component

    """

    params = {'wait_time': 0.3, 'velocity': 0.2, 'travel_z': env['travel_z']}
    communicate_parameters(server, params)

    server.recv(header='imaging_pose')
    server.send(str(list(env['part_area'][0])))

    # TODO: sent nans or something to end the program in UR controller

    for i in range(2):
        server.recv(header='grab_pose')
        server.send(str(list(env['build_area'][0])))

        server.recv(header='place_pose')
        server.send(str(list(env['build_area'][1])))

        server.recv(header='continue')
        if i >= 1:
            cont = 0
        else:
            cont = 1
        server.send(str([cont]))


def communicate_parameters(server, params):

    for _ in range(len(params.keys())):
        msg = server.recv()
        response = str([params[msg]])
        server.send(response)


def _add_to_pose(pose, name, val):
    """ Add value to pose

    Edits 'pose' in-place.

    Parameters
    ----------
    pose : list
        Pose as [x,y,z,rx,ry,rz].
    name : str
        One of: x,y,z,rx,ry,rz

    val : float

    """

    if name == 'z':
        pose[2] += val
    else:
        raise NotImplementedError('Addition for "{}" not implemented'.format(name))