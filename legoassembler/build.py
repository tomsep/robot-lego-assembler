from __future__ import division
from __future__ import print_function
import time
import numpy as np
from math import radians, sin, cos, atan2
from copy import deepcopy
import json

from legoassembler.vision import MachineVision, NoMatches


def teach_platform(rob, gripper, tcp):

    rob.set_tcp(tcp['gripper'])
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

    rob.grip(closed=gripper['open_tight'])

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


def preview_taught_platform(rob, tcp, calib, travel_height, gripper):

    rob.set_tcp(tcp['gripper'])
    wait = 0.1
    vel = 0.3
    a = 0.1
    poses = calib['taught_poses']
    rob.popup('Press continue to start preview', blocking=True)

    pose = rob.get_tcp()
    pose[2] = poses['build'][0][2] + travel_height
    rob.movel(pose, v=vel, a=a)

    # Open gripper
    rob.grip(closed=gripper['open'])

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


def calibrate_camera(rob, mv, gripper, tcp, travel_height, calib,
                     brick2x2_side_mm, color):

    rob.set_tcp(tcp['gripper'])
    wait = 0.1
    vel = 1.3
    a = 0.4
    imaging_area = calib['taught_poses']['build'][0]

    rob.popup('Press continue to start camera calibration', blocking=True)

    # Goto starting height
    pose = rob.get_tcp()
    pose[2] = imaging_area[2] + travel_height
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    # Open gripper
    rob.grip(closed=gripper['open'])

    pose = deepcopy(imaging_area)
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)
    time.sleep(wait)
    rob.movel(imaging_area, v=vel, a=a)
    time.sleep(wait * 2)
    rob.movel(pose, v=vel, a=a)

    rob.set_tcp(tcp['camera'])
    rob.movej(pose, v=vel, a=a)
    mv.calibrate(brick2x2_side_mm, color)  # Take calibration image

    # Move aside: x, y and yaw
    rotvec = rob.rpy2rotvec([0, 0, radians(0)])
    pose = [0.07, 0.05, 0] + rotvec
    rob.movej(pose, v=vel, a=a, relative=True)
    time.sleep(wait)

    target = [None, None, None, 'parallel_to_x', color, '2x2']

    # Find match and move above it
    _find_brick_iteratively(rob, mv, target, tcp, imaging_area, travel_height, vel, a,
                            max_diff=0.7)

    # Touch the match
    pose = rob.get_tcp()
    pose[2] = imaging_area[2]
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait * 2)
    pose[2] = travel_height
    rob.movel(pose, v=vel, a=a)
    time.sleep(wait)

    rob.popup('Camera calibration finished!')
    print('Camera calibration finished!')


def _wiggle(robot, max_mm, target_pose):
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


def build(rob, mv, gripper, tcp, platf_calib, plan, travel_height, unit_brick_dims,
          load_state=False):

    wait = 0.0
    vel = 1.5
    a = 0.6 * 2

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
    rob.set_tcp(tcp['gripper'])

    imaging_area = _midpoint_of_poses(platf_calib['taught_poses']['part'][0],
                                      platf_calib['taught_poses']['part'][1])
    imaging_height = imaging_area[2] + travel_height

    # Goto starting height above
    _move_to_height(rob, imaging_height, vel, a, movelinear=True)


    allowed_pickup_area = [platf_calib['taught_poses']['part'][0],
                           platf_calib['taught_poses']['part'][1]]

    for i in range(state['current_index'], len(state['plan'])):

        target = plan[i]

        # Open gripper
        rob.grip(closed=gripper['open'])

        # Find match and move above it
        _find_brick_iteratively(rob, mv, target, tcp, imaging_area, travel_height, vel,
                                a, max_diff=0.7, allowed_rect_area=allowed_pickup_area)

        #  ----Grab the match----
        # Move just a little above the brick
        z = imaging_area[2] + unit_brick_dims['base_height'] * 1.5
        _move_to_height(rob, z, vel, a, movelinear=True)

        # Move slower all the way down
        z = imaging_area[2] + unit_brick_dims['base_height']
        _move_to_height(rob, z, vel / 2, a / 3, movelinear=True)

        # Grip and go back up
        rob.grip(closed=gripper['closed'], force=gripper['force'])
        _move_to_height(rob, travel_height, vel, a, movelinear=True)

        # Place on platform
        _place_on_platform(rob, platf_calib['taught_poses']['build'],
                           target, travel_height, vel, a, unit_brick_dims)

        rob.grip(closed=gripper['open'])

        # Save state
        state['current_index'] = i + 1
        with open(state_fname, 'w') as f:
            f.write(json.dumps(state))

        _move_to_height(rob, travel_height, vel, a, movelinear=True)


def _place_on_platform(rob, build_platf, target, travel_height, vel, a, unit_brick_dims):

    # Get grid step size for the build platform
    corners = np.array(build_platf)[:, :2]
    x_step, y_step = _actual_step_width(corners, unit_brick_dims['side'])

    # Build platform origin corner
    origin = deepcopy(build_platf[0])
    points = [x[:2] for x in build_platf]
    platf_angle = rect_angle(points) - radians(180)
    origin[3:] = rob.rpy2rotvec(rob.rotvec2rpy(origin[3:])[:-1] + [platf_angle])

    # Compute target point in build platform
    # TODO: What if platform axis change?
    p2 = [-target[1] * x_step, target[2] * y_step, 0, 0, 0, 0]
    pose = rob.pose_trans(origin, p2)
    # Preferred yaw (to prevent rotating too much)
    preferred_yaw = rob.rotvec2rpy(build_platf[0][3:])[-1]
    pose = _untangle_rz(rob, pose, preferred_angle=preferred_yaw)

    # move above
    _move_to_height(rob, travel_height, vel, a, pose)

    # move just above brick
    target_z = origin[2] + (target[0] - 1) * unit_brick_dims['base_height']
    z = target_z + unit_brick_dims['base_height'] * 1.05
    _move_to_height(rob, z, vel, a, pose, movelinear=True)

    # Move slower to place
    _move_to_height(rob, target_z, vel / 2, a / 3, pose, movelinear=True)


def _move_to_height(rob, height, vel, a, pose=None, movelinear=False):
    if pose:
        pose = deepcopy(pose)
        pose[2] = height
    else:
        pose = rob.get_tcp()
        pose[2] = height

    if movelinear:
        rob.movel(pose, v=vel, a=a)
    else:
        rob.movej(pose, v=vel, a=a)


def _actual_step_width(points, ideal):

    if type(points) != np.ndarray:
        points = np.array(points)

    point_dim = np.shape(points)[1]
    if not point_dim == 2:
        raise ValueError('Points are not 2D')

    # Add 0 3rd vector component
    points = np.append(points, np.zeros((4, 1)), axis=1)

    # TODO: test if close to rectangle

    # Find y and x direction vectors by trying both of last 2 calib points
    edge1 = points[2] - points[0]
    edge2 = points[3] - points[0]
    cross = np.cross(edge1, edge2)
    if cross[0] > 0:
        axis_x = edge2
        axis_y = edge1
    else:
        axis_x = edge1
        axis_y = edge2

    axis_x_norm = np.linalg.norm(axis_x)
    axis_y_norm = np.linalg.norm(axis_y)
    axis_x_dim = round(axis_x_norm / ideal, ndigits=0)
    axis_y_dim = round(axis_y_norm / ideal, ndigits=0)

    actual_x_step = axis_x_norm / axis_x_dim
    actual_y_step = axis_y_norm / axis_y_dim

    return actual_x_step, actual_y_step


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


def _untangle_rz(rob, pose, preferred_angle):
    """ Choose closest rz rotation (by adding +180 deg) to preferred_angle

    Parameters
    ----------
    rob
    pose
    preferred_angle : float
        Radians -2pi..2pi.
        Angle RZ to use as basis for choosing whether to add +180 deg or not.

    Returns
    -------

    """

    rpy = rob.rotvec2rpy(pose[3:])
    if abs(rpy[2] - preferred_angle) > radians(90):
        rpy[2] = rpy[2] + radians(180)
        pose = deepcopy(pose)
        pose[3:] = rob.rpy2rotvec(rpy)
    return pose


def _find_brick_iteratively(rob, mv, target, tcp, imaging_area, travel_height,
                            vel, a, max_diff=0.7, allowed_rect_area=None):
    """ Move gripper above brick that matches best the target given

    At the end of the function TCP is set back to 'gripper'.
    Returns None when above the match with maximum  of 'max_diff' deviation.

    """

    # Go above imaging area
    rob.set_tcp(tcp['camera'])
    pose = deepcopy(imaging_area)
    pose[2] += travel_height
    rob.movej(pose, v=vel, a=a)

    if target[5] == '2x2':
        size = (2, 2)
    else:
        size = (2, 4)

    match = {'x': 255, 'y': 255, 'angle': 255}
    while abs(match['x']) > max_diff or abs(match['y']) > max_diff:
        try:
            if target[3] != 'parallel_to_y':
                # Turn to grab the brick in alternative direction
                use_max_edge = True
            else:
                use_max_edge = False
            match = mv.find_brick(target[4], size, margin=0.2,
                                  use_max_edge=use_max_edge, draw=True)
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

        if allowed_rect_area:

            is_it = _is_within_rect(curr_pose_xy, allowed_rect_area[0],
                                    allowed_rect_area[1])
            if not is_it:
                # Go back to above imaging area
                pose = deepcopy(imaging_area)
                pose[2] += travel_height
                rob.movej(pose, v=vel, a=a)
                continue

        rob.movej(pose_, v=vel, a=a, relative=True)

    # Move gripper where the camera is
    pose = rob.get_tcp()
    rob.set_tcp(tcp['gripper'])
    rob.movej(pose, v=vel, a=a)

    return
