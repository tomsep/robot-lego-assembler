from __future__ import division
from __future__ import print_function
import time
import numpy as np
from math import radians, sin, cos
from copy import deepcopy

from vision import MachineVision


def teach_platform(rob):

    build_area = []

    def _level_pose(p_):
        # Make TCP pose level with the ground:
        # set roll=180, set pitch=0 and yaw unchanged (deg)
        rotvec = p_[3:]
        rpy = rob.rotvec2rpy(rotvec)
        rpy = [radians(180), 0, rpy[2]]
        rotvec = rob.rpy2rotvec(rpy)
        p_[3:] = rotvec
        return p_

    rob.popup('Hit continue to start guided calibration procedure.', blocking=True)
    rob.popup('Continue to initialize gripper.', blocking=True)

    rob.grip(closed=76)

    msg = 'Place 2x2 block on one corner of the build platform.' \
              ' Guide the arm to grab the block.'
    rob.teachmode(msg)

    build_area.append(_level_pose(rob.get_tcp()))

    msg = 'Place 2x2 block on the diagonal opposite corner of the build platform.' \
              ' Guide the arm to grab the block.'
    rob.teachmode(msg)
    build_area.append(_level_pose(rob.get_tcp()))

    msg = 'Move the gripper to middle (ground level)' \
              ' of the pickup platform.'
    rob.teachmode(msg)
    part_area = _level_pose(rob.get_tcp())

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
    rob.movel(pose, v=vel, a=a)

    # Open gripper
    rob.grip(closed=76)

    pose = deepcopy(poses['build'][0])
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(poses['build'][0], v=vel, a=a)
    time.sleep(wait)
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    pose = deepcopy(poses['build'][1])
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(poses['build'][1], v=vel, a=a)
    time.sleep(wait)
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    pose = deepcopy(poses['part'])
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(poses['part'], v=vel, a=a)
    time.sleep(wait)
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    rob.popup('Preview finished!')
    print('Preview finished!')


def calibrate_camera(rob, mv, travel_height, calib, brick2x2_side_mm, color):

    wait = 0.1
    vel = 0.5
    a = 0.1
    test_pose = calib['taught_poses']['build'][0]

    rob.popup('Press continue to start camera calibration', blocking=True)

    # Goto starting height
    pose = rob.get_tcp()
    pose[2] = test_pose[2] + travel_height
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    # Open gripper
    rob.grip(closed=76)

    pose = deepcopy(test_pose)
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(test_pose, v=vel, a=a)
    time.sleep(wait * 2)
    rob.movel(pose, v=vel, a=a)

    # Take calibration image
    mv.calibrate(brick2x2_side_mm, color)

    # Move aside: x, y and yaw
    rotvec = rob.rpy2rotvec([0, 0, radians(-45)])
    pose = [-0.03, 0.02, 0] + rotvec
    rob.movej(pose, v=vel, a=a, relative=True)
    time.sleep(wait)

    # Take new image
    match = mv.find_brick(color, (2, 2), margin=0.2, draw=True)

    # Transform x and y from image's frame to robot's frame
    rpy = rob.rotvec2rpy(rob.get_tcp()[3:])  # rpy = [roll, pitch, yaw]
    p1 = [0, 0, 0] + rob.rotvec2rpy([0, 0, rpy[2]])
    p2 = [match['x']/1000, match['y']/1000, 0] + rob.rotvec2rpy([0, 0, 0])
    trans_xyz = rob.pose_trans(p1, p2)[:3]

    # Add yaw to correct angle
    pose = trans_xyz + rob.rpy2rotvec([0, 0, -match['angle']])

    # Move above the match
    rob.movej(pose, v=vel, a=a, relative=True)
    time.sleep(wait)

    # Touch the match
    pose = rob.get_tcp()
    pose[2] = test_pose[2]
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait * 2)
    pose[2] = travel_height
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    rob.popup('Camera calibration finished!')
    print('Camera calibration finished!')