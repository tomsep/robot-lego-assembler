from __future__ import division
from __future__ import print_function
import time
import numpy as np
from math import radians, sin, cos
from copy import deepcopy

from legoassembler.vision import MachineVision, NoMatches

GOPEN = 61.5
GCLOSED = 70
FORCE = 55
TCP = [0, 0, 0.193, 0, 0, 0]
TCP_CAM = [0, -0.0625, 0.193, 0, 0, 0]

def teach_platform(rob):

    rob.set_tcp(TCP)
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

    rob.grip(closed=GOPEN)

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

    rob.set_tcp(TCP)
    wait = 0.1
    vel = 0.3
    a = 0.1
    poses = calib['taught_poses']
    rob.popup('Press continue to start preview', blocking=True)

    pose = rob.get_tcp()
    pose[2] = poses['build'][0][2] + travel_height
    rob.movel(pose, v=vel, a=a)

    # Open gripper
    rob.grip(closed=GOPEN)

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

    rob.set_tcp(TCP)
    wait = 0.1
    vel = 1.3
    a = 0.4
    test_pose = calib['taught_poses']['build'][0]

    rob.popup('Press continue to start camera calibration', blocking=True)

    # Goto starting height
    pose = rob.get_tcp()
    pose[2] = test_pose[2] + travel_height
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    # Open gripper
    rob.grip(closed=GOPEN)

    pose = deepcopy(test_pose)
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(test_pose, v=vel, a=a)
    time.sleep(wait * 2)
    rob.movel(pose, v=vel, a=a)

    rob.set_tcp(TCP_CAM)
    rob.movej(pose, v=vel, a=a)
    mv.calibrate(brick2x2_side_mm, color)  # Take calibration image

    # Move aside: x, y and yaw
    rotvec = rob.rpy2rotvec([0, 0, radians(0)])
    pose = [0.07, 0.05, 0] + rotvec
    rob.movej(pose, v=vel, a=a, relative=True)
    time.sleep(wait)

    # Find the brick
    match = mv.find_brick(color, (2, 2), margin=0.2, draw=True)
    print(match)
    while abs(match['x']) > 0.7 or abs(match['y']) > 0.7:
        xoff = match['x'] / 1000
        yoff = match['y'] / 1000
        angle_off = match['angle']
        rpy_ = rob.rotvec2rpy(rob.get_tcp()[3:])  # rpy = [roll, pitch, yaw]
        p1 = [0, 0, 0] + rob.rotvec2rpy([0, 0, rpy_[2] - radians(180)])
        p2 = [xoff, yoff, 0] + rob.rotvec2rpy([0, 0, 0])
        pose_ = rob.pose_trans(p1, p2)[:3] + [0, 0, angle_off]

        rob.movej(pose_, v=vel, a=a, relative=True)

        match = mv.find_brick(color, (2, 2), margin=0.2, draw=True)
        print(match)
        time.sleep(wait)

    def _imaging(on_, xoff_, yoff_):
        rpy_ = rob.rotvec2rpy(rob.get_tcp()[3:])  # rpy = [roll, pitch, yaw]
        p1 = [0, 0, 0] + rob.rotvec2rpy([0, 0, rpy_[2] - radians(180)])
        if on_ is False:
            xoff_ = -xoff_
            yoff_ = -yoff_
        p2 = [xoff_, yoff_, 0] + rob.rotvec2rpy([0, 0, 0])
        pose_ = rob.pose_trans(p1, p2)[:3] + [0, 0, 0]
        rob.movej(pose_, v=vel, a=a, relative=True)
        time.sleep(wait)
        return pose_

    rob.set_tcp(TCP)
    _imaging(False, -TCP_CAM[0], -TCP_CAM[1])
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


def test_camera(rob, mv, travel_height, calib, color):

    rob.set_tcp(TCP)
    wait = 0.0
    vel = 1
    a = 0.3
    imaging_area = calib['taught_poses']['part']
    place_area = calib['taught_poses']['build'][0]

    rob.popup('Press continue to start test', blocking=True)

    # Goto starting height above
    pose = rob.get_tcp()
    pose[2] = imaging_area[2] + travel_height
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    def _imaging(on_, xoff_, yoff_):
        rpy_ = rob.rotvec2rpy(rob.get_tcp()[3:])  # rpy = [roll, pitch, yaw]
        p1 = [0, 0, 0] + rob.rotvec2rpy([0, 0, rpy_[2] - radians(180)])
        if on_ is False:
            xoff_ = -xoff_
            yoff_ = -yoff_
        p2 = [xoff_, yoff_, 0] + rob.rotvec2rpy([0, 0, 0])
        pose_ = rob.pose_trans(p1, p2)[:3] + [0, 0, 0]
        rob.movej(pose_, v=vel, a=a, relative=True)
        time.sleep(wait)
        return pose_

    while True:

        # Go above imaging area
        rob.set_tcp(TCP)
        pose = deepcopy(imaging_area)
        pose[2] += travel_height
        rob.movej(pose, v=vel, a=a)

        rob.set_tcp(TCP_CAM)
        # Open gripper
        rob.grip(closed=GOPEN)

        match = {'x': 255, 'y': 255, 'angle': 255}
        while abs(match['x']) > 0.7 or abs(match['y']) > 0.7:
            try:
                match = mv.find_brick(color, (2, 2), margin=0.2, draw=True)
            except NoMatches:
                continue

            print(match)
            xoff = match['x'] / 1000
            yoff = match['y'] / 1000
            angle_off = match['angle']
            rpy_ = rob.rotvec2rpy(rob.get_tcp()[3:])  # rpy = [roll, pitch, yaw]
            p1 = [0, 0, 0] + rob.rotvec2rpy([0, 0, rpy_[2] - radians(180)])
            p2 = [xoff, yoff, 0] + rob.rotvec2rpy([0, 0, 0])
            pose_ = rob.pose_trans(p1, p2)[:3] + [0, 0, angle_off]
            rob.movej(pose_, v=vel, a=a, relative=True)
            time.sleep(wait)

        rob.set_tcp(TCP)
        _imaging(False, -TCP_CAM[0], -TCP_CAM[1])
        time.sleep(wait)

        # Grab the match
        pose = rob.get_tcp()
        pose[2] = imaging_area[2] + 0.02
        rob.movel(pose, v=vel, a=a)
        time.sleep(wait * 2)
        rob.grip(closed=GCLOSED, force=FORCE)
        pose[2] = travel_height
        rob.movel(pose, v=vel, a=a)
        time.sleep(wait)

        # Place
        pose = deepcopy(place_area)
        pose[2] += travel_height
        rob.movej(pose, v=vel, a=a)
        pose_ = deepcopy(place_area)
        pose_[2] += 0.023
        rob.movel(pose_)

        place_block(rob, target_z=place_area[2], target_pose=place_area)

        rob.grip(closed=GOPEN)
        rob.movel(pose)




def place_block(rob, target_z, target_pose, max_mm=1.5):

    force = 37  # Newtons
    rob.force_mode_tool_z(force, 2)
    while abs(rob.get_tcp()[2] - target_z) > max_mm / 1000:
        wiggle(rob, 1, target_pose)
        rob.force_mode_tool_z(force, 1)


def wiggle(robot, max_mm, target_pose):
    x, y = np.random.rand(2) * max_mm
    xs, ys = np.random.rand(2)  # random sign
    tcp = deepcopy(target_pose)
    if xs > 0.5:
        x *= -1
    if ys > 0.5:
        y *= -1
    tcp[0] += x / 1000
    tcp[1] += y / 1000
    tcp[2] = robot.get_tcp()[2]
    robot.movej(tcp, v=0.1, a=0.05)