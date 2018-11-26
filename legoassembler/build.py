from __future__ import division
from __future__ import print_function
import time
import numpy as np
from math import radians, sin, cos, atan2
from copy import deepcopy
import json

from legoassembler.vision import MachineVision, NoMatches

GOPEN = 60
GOPEN_TIGHT = 65
GCLOSED = 69
FORCE = 55
TCP = [0, 0, 0.193, 0, 0, 0]
TCP_CAM = [0, -0.0625, 0.193, 0, 0, 0]

def teach_platform(rob):

    rob.set_tcp(TCP)
    build_area = []
    part_area = []

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

    rob.grip(closed=GOPEN_TIGHT)

    msg = 'Place 2x2 block on one corner of the build platform.' \
              ' Guide the arm to grab the block.'
    rob.teachmode(msg)

    build_area.append(_level_pose(rob.get_tcp()))

    msg = 'Place 2x2 block on the diagonal opposite corner of the build platform.' \
              ' Guide the arm to grab the block.'
    rob.teachmode(msg)
    build_area.append(_level_pose(rob.get_tcp()))

    msg = 'Repeat for another two corners. Corner 1.'
    rob.teachmode(msg)
    build_area.append(_level_pose(rob.get_tcp()))

    msg = 'Last corner.'
    rob.teachmode(msg)
    build_area.append(_level_pose(rob.get_tcp()))

    msg = 'Move the gripper to corner (ground level)' \
          ' of the pickup platform.'
    rob.teachmode(msg)
    part_area.append(_level_pose(rob.get_tcp()))

    msg = 'Move the gripper to diagonal opposite corner (ground level)' \
          ' of the pickup platform.'
    rob.teachmode(msg)
    part_area.append(_level_pose(rob.get_tcp()))

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

    pose = deepcopy(poses['build'][2])
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(poses['build'][2], v=vel, a=a)
    time.sleep(wait)
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    pose = deepcopy(poses['build'][3])
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(poses['build'][3], v=vel, a=a)
    time.sleep(wait)
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    pose = deepcopy(poses['part'][0])
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(poses['part'][0], v=vel, a=a)
    time.sleep(wait)
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    pose = deepcopy(poses['part'][1])
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(poses['part'][1], v=vel, a=a)
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

    imaging_area = _midpoint_of_poses(calib['taught_poses']['part'][0],
                                      calib['taught_poses']['part'][1])
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
    factor = 1.2
    rob.force_mode_tool_z(force / factor, 1)
    while True:
        deviation = abs(rob.get_tcp()[2] - target_z)
        if deviation <= max_mm / 1000:
            break

        elif deviation > 7 / 1000:
            wiggle(rob, 0.2, target_pose)
            rob.force_mode_tool_z(force / factor, 1)
        else:
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


def build(rob, mv, platf_calib, plan, travel_height, load_state=False):
    wait = 0.0
    vel = 1.5
    a = 0.6

    state_fname = 'build_state.json'
    if load_state:
        with open(state_fname, 'r') as f:
            state = json.loads(f.read())
        if state['current_index'] > len(state['plan']):
            print('Loaded build already finished!')
            return
    else:
        state = {'plan': plan, 'current_index': 0}


    plan = state['plan']

    rob.popup('Continue to start building', blocking=True)
    rob.set_tcp(TCP)

    imaging_area = _midpoint_of_poses(platf_calib['taught_poses']['part'][0],
                                      platf_calib['taught_poses']['part'][1])

    # Goto starting height above
    pose = rob.get_tcp()
    pose[2] = imaging_area[2] + travel_height
    rob.movel(pose, v=vel, a=a)

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

    for i in range(state['current_index'], len(state['plan'])):

        target = plan[i]
        if target[5] == '2x2':
            size = (2, 2)
        else:
            size = (2, 4)
        # Go above imaging area
        rob.set_tcp(TCP_CAM)
        pose = deepcopy(imaging_area)
        pose[2] += travel_height
        rob.movej(pose, v=vel, a=a)

        # Open gripper
        rob.grip(closed=GOPEN)

        match = {'x': 255, 'y': 255, 'angle': 255}
        while abs(match['x']) > 0.7 or abs(match['y']) > 0.7:
            try:
                match = mv.find_brick(target[4], size, margin=0.2, draw=True)
            except NoMatches:
                continue

            print(match)
            xoff = match['x'] / 1000
            yoff = match['y'] / 1000
            angle_off = match['angle']
            rpy_ = rob.rotvec2rpy(rob.get_tcp()[3:])  # rpy = [roll, pitch, yaw]
            p1 = [0, 0, 0] + rob.rotvec2rpy([0, 0, rpy_[2] - radians(180)])
            p2 = [xoff, yoff, 0] + rob.rotvec2rpy([0, 0, 0])

            # If square use the closest angle
            if size[0] == size[1]:
                if angle_off > radians(45):
                    angle_off -= radians(90)
                elif angle_off < radians(-45):
                    angle_off += radians(90)
            pose_ = rob.pose_trans(p1, p2)[:3] + [0, 0, angle_off]

            curr_pose_xy = rob.get_tcp()[:2]
            curr_pose_xy[0] += pose_[0]
            curr_pose_xy[1] += pose_[1]

            is_it = _is_within_rect(curr_pose_xy, platf_calib['taught_poses']['part'][0],
                                      platf_calib['taught_poses']['part'][1])
            if not is_it:
                # Go back to above imaging area
                pose = deepcopy(imaging_area)
                pose[2] += travel_height
                rob.movej(pose, v=vel, a=a)
                continue

            rob.movej(pose_, v=vel, a=a, relative=True)
            time.sleep(wait)

        rob.set_tcp(TCP)
        _imaging(False, -TCP_CAM[0], -TCP_CAM[1])
        time.sleep(wait)

        # Grab the match
        pose = rob.get_tcp()
        pose[2] = imaging_area[2] + 0.02
        rob.movel(pose, v=vel, a=a)
        rob.force_mode_tool_z(25, 0.5)
        rob.grip(closed=GCLOSED, force=FORCE)
        pose[2] = travel_height
        rob.movel(pose, v=vel, a=a)
        time.sleep(wait)

        # Place
        if target[3] != 'parallel_to_y':
            turn = radians(-90)
        else:
            turn = 0
        _place_on_platform(rob, platf_calib['taught_poses']['build'],
                           target, travel_height, vel, a, turn)

        rob.grip(closed=GOPEN)

        # Save state
        state['current_index'] = i + 1
        with open(state_fname, 'w') as f:
            f.write(json.dumps(state))

        pose = rob.get_tcp()
        pose[2] = travel_height
        rob.movel(pose, v=vel, a=a)


def pickup_demo(rob, mv, travel_height, platf_calib, colors):
    wait = 0.0
    vel = 1.5
    a = 0.6

    drop_off_height = 0.25

    rob.popup('Continue to start pickup demo', blocking=True)
    rob.set_tcp(TCP)

    imaging_area = _midpoint_of_poses(platf_calib['taught_poses']['part'][0],
                                      platf_calib['taught_poses']['part'][1])

    # Goto starting height above
    pose = rob.get_tcp()
    pose[2] = imaging_area[2] + travel_height
    rob.movel(pose, v=vel, a=a)

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
        rob.set_tcp(TCP_CAM)
        pose = deepcopy(imaging_area)
        pose[2] += travel_height
        rob.movej(pose, v=vel, a=a)

        cidx = np.random.randint(0, len(colors))
        color = colors[cidx]

        # Open gripper
        rob.grip(closed=GOPEN)

        match = {'x': 255, 'y': 255, 'angle': 255}
        while abs(match['x']) > 0.7 or abs(match['y']) > 0.7:
            try:
                match = mv.find_brick(color, (2, 2), margin=0.2, draw=True)
            except NoMatches:
                # Reselect color
                cidx = np.random.randint(0, len(colors))
                color = colors[cidx]
                continue

            print(match)
            xoff = match['x'] / 1000
            yoff = match['y'] / 1000
            angle_off = match['angle']
            rpy_ = rob.rotvec2rpy(rob.get_tcp()[3:])  # rpy = [roll, pitch, yaw]
            p1 = [0, 0, 0] + rob.rotvec2rpy([0, 0, rpy_[2] - radians(180)])
            p2 = [xoff, yoff, 0] + rob.rotvec2rpy([0, 0, 0])
            pose_ = rob.pose_trans(p1, p2)[:3] + [0, 0, angle_off]

            curr_pose_xy = rob.get_tcp()[:2]
            curr_pose_xy[0] += pose_[0]
            curr_pose_xy[1] += pose_[1]

            is_it = _is_within_rect(curr_pose_xy, platf_calib['taught_poses']['part'][0],
                                      platf_calib['taught_poses']['part'][1])
            if not is_it:
                # Go back to above imaging area
                pose = deepcopy(imaging_area)
                pose[2] += travel_height
                rob.movej(pose, v=vel, a=a)
                continue

            rob.movej(pose_, v=vel, a=a, relative=True)
            time.sleep(wait)

        rob.set_tcp(TCP)
        _imaging(False, -TCP_CAM[0], -TCP_CAM[1])
        time.sleep(wait)

        # Grab the match
        pose = rob.get_tcp()
        pose[2] = imaging_area[2] + 0.02
        rob.movel(pose, v=vel, a=a)
        rob.force_mode_tool_z(25, 0.5)
        rob.grip(closed=GCLOSED, force=FORCE)
        pose[2] = drop_off_height
        rob.movel(pose, v=vel, a=a)
        time.sleep(wait)

        # Place to bucket above build[1] corner
        pose = deepcopy(platf_calib['taught_poses']['build'][1])
        pose[2] += drop_off_height
        rob.movej(pose, v=vel, a=a)
        rob.grip(closed=GOPEN)

def _place_on_platform(rob, build_platf, target, travel_height, vel, a, turn=0):
    # Use build_platf[0] as origin of the platform. X and Y towards corner [1]
    x_sign, y_sign = (1, 1)
    if build_platf[0][0] - build_platf[1][0] > 0:
        x_sign *= -1
    if build_platf[0][1] - build_platf[1][1] > 0:
        y_sign *= -1

    origin_pose = deepcopy(build_platf[0])
    rpy = rob.rotvec2rpy(origin_pose[3:])
    points = [x[:2] for x in build_platf]
    rpy[2] = np.unwrap([rect_angle(points) - radians(180)])[0]
    rotvec = rob.rpy2rotvec(rpy)
    origin_pose[3:] = rotvec

    # Platform coords to global
    rpy_ = rob.rotvec2rpy(origin_pose[3:])  # rpy = [roll, pitch, yaw]
    p1 = [0, 0, 0] + rob.rotvec2rpy([0, 0, rpy_[2] - radians(180)])
    p2 = [target[1] / 1000, target[2] / 1000, 0] + rob.rotvec2rpy([0, 0, radians(90)])
    target_rel_pose = rob.pose_trans(p1, p2)[:3] + [0, 0, 0]
    target_pose = deepcopy(origin_pose)
    target_pose[0] += target_rel_pose[0]
    target_pose[1] += target_rel_pose[1]
    target_pose[2] += target[0] / 1000
    target_pose = rob.pose_trans(target_pose, [0, 0, 0] + rob.rpy2rotvec([0, 0, turn]))

    # target_pose = deepcopy(origin_pose)
    # target_pose[0] += target[1] * x_sign / 1000
    # target_pose[1] += target[2] * y_sign / 1000
    # target_pose[2] += target[0] / 1000

    pose = deepcopy(target_pose)
    pose[2] = travel_height
    rob.movej(pose, v=vel, a=a)

    # Go just above the targetz
    target_z = target_pose[2]
    pose = deepcopy(target_pose)
    pose[2] = target_z + 0.022
    rob.movel(pose, v=vel, a=a)

    place_block(rob, target_z=target_z, target_pose=target_pose)


def _midpoint_of_poses(p1, p2):
    # xy midpoint, rotation and z same as p1.
    pose = deepcopy(p1)
    pose[0] = (p1[0] + p2[0]) / 2
    pose[1] = (p1[1] + p2[1]) / 2
    return pose


def _is_within_rect(xy, p1, p2):
    """ If xy is withing rectangle defined by p1, p2

    """

    for i in range(2):
        if p1[i] <= xy[i] <= p2[i] or p2[i] <= xy[i] <= p1[i]:
            continue
        else:
            return False

    return True


def rect_angle(points):
    # points: [p1, p_diag_to_p1, p2, p_diag_to_p2]; 2D points
    # return angle in right-handed coordinate system

    points = [np.array(x) for x in points]
    if len(points[0]) == 2:
        points = [np.append(x, 0) for x in points]

    # Find x-axis
    x_vec = points[2] - points[0]
    y_vec = points[3] - points[0]
    cross = np.cross(x_vec, y_vec)
    if np.sign(cross[2]) == -1:
        x_vec = y_vec

    # Angle around z w.r.t global x-axis
    angle = atan2(x_vec[1], x_vec[0]) - atan2(0, 1)
    return np.unwrap([angle])[0]