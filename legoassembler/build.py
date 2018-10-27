from __future__ import division
import json


def paltform_calibration(server, z_clearance):
    """ Runs platform calibration sequence to create new environment dictionary

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
        Environment dict

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
    for _ in range(2):
        pose = json.loads(server.recv()[1:])
        print('Received part area pose: {}'.format(pose))
        _append_xyz(part_area, pose)

    # travel z
    max_z = max(x[2] for x in build_area + part_area)
    travel_z = max_z + z_clearance

    environment = {'build_area': tuple(build_area), 'part_area': tuple(part_area),
                   'travel_z': travel_z}
    return environment


def camera_calibration(server, env, mv):
    """ Runs camera calibration sequence

    Parameters
    ----------
    server : URServer
    env : dict
        Environment dict.
    mv : ???
        Machine vision component.

    Returns
    -------
    ???
        Camera calibration.

    """
    raise NotImplementedError()


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

