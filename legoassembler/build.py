from __future__ import division
from __future__ import print_function
import time
import numpy as np
from math import radians, sin, cos, atan2
from copy import deepcopy
import json
from threading import Thread

from legoassembler.vision import MachineVision, NoMatches


def teach_platform(rob, gripper, tcp):
    """ Teach platform procedure.

    Poses taught are leveled so that the all the poses' z axis' are orthogonal
    to the table that is assumed to be orthogonal to the global z axis.

    Parameters
    ----------
    rob : Robot
    gripper : dict
    tcp : dict

    Returns
    -------
    dict
        Taught poses as dictionary. build_area has 4 corner poses and part_area 2 corners,
        i.e. list of lengths 4 and 2.
        {'taught_poses': {'build': build_area, 'part': part_area}}

    """

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
    """ Run the robot slowly through all taught poses.

    Parameters
    ----------
    rob : Robot
    tcp : dict
    calib : dict
    travel_height : float
    gripper : dict

    """

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
                     brick2x2_side_mm, color, brick_base_height):
    """ Calibrates MachineVision object by imaging a 2x2 brick.

    Calibration brick's position is assumed to be the first taught build platform
    pose.

    1. Touch the origin pose (first calibrated corner).
    2. Take picture for calibration.
    3. Move a bit aside.
    4. Test calibration by trying to guide the arm back to the calibration brick
        using only machine vision.

    Parameters
    ----------
    rob : Robot
    mv : MachineVision
    gripper : dict
    tcp : dict
    travel_height : float
    calib : dict
    brick2x2_side_mm : float
        Ideal side length of the 2x2 brick.
    color : str
        Name of the color used for calibration brick.
    brick_base_height : float
        In mm the height of a brick without its studs.

    """

    rob.set_tcp(tcp['gripper'])
    wait = 0.1
    vel = 1.3
    a = 0.4
    imaging_area = calib['taught_poses']['build'][0]

    rob.popup('Press continue to start camera calibration', blocking=True)

    # Goto starting height
    pose = rob.get_tcp()
    pose[2] = imaging_area[2] + travel_height - brick_base_height / 1000
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
                            max_diff=0.7, use_color_vision=True, use_model=False)

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



def build(rob, mv, gripper, tcp, platf_calib, plan, travel_height, unit_brick_dims, finger_region,
          failure_detection_on, use_color_vision, use_model, load_state=False):
    """ Build a structure based on list of instructions.

    Instrutions are expected to be a list
    [float, float, float, str, str, str]  # types
    [z, x, y, brick_orientation, color_name, size]  # value names
    where z, x and y are width units (1 unit := one DUPLO stud) that are later converted
    to millimeters using 'unit_brick_dims'. Brick orientation is a string 'parallel_to_x'
    or 'parallel_to_y', and color_name is name of the color e.g. "red". Size is
    the brick's size e.g. "2x2" or "2x4".

    A progress state file is saved and updated after each placed brick. The state can
    be loaded later if for example the system crashes.


    Parameters
    ----------
    rob : Robot
    mv : MachineVision
    gripper : dict
    tcp : dict
    platf_calib : dict
    plan : list
        Build plan as a list of instructions.
    travel_height : float
    unit_brick_dims : dict
    finger_region : list
        Region of where the gripped brick in the image. A pixel ranges of height and width.
    failure_detection_on : bool
        Whether to use gripping failure detection.
    load_state : bool
        If previous state should be load from disk.

    """

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

    step_idx = state['current_index']
    while True:
        try:
            target = plan[step_idx]
        except IndexError:
            break  # reached the end of plan

        # Open gripper if needed
        if abs(rob.gripper_actual_pos() - gripper['open']) > 1:
            # Not gripped anything. Open and try again.
            rob.grip(closed=gripper['open'])

        # Find match and move above it
        _find_brick_iteratively(rob, mv, target, tcp, imaging_area, travel_height, vel,
                                a, max_diff=0.7, allowed_rect_area=allowed_pickup_area,
                                use_color_vision=use_color_vision, use_model=use_model)

        #  ----Grab the match----
        # Move just a little above the brick
        z = imaging_area[2] + unit_brick_dims['base_height'] * 1.5
        _move_to_height(rob, z, vel, a, movelinear=True)

        # Move slower all the way down
        z = imaging_area[2] + unit_brick_dims['base_height']
        _move_to_height(rob, z, vel / 2, a / 3, movelinear=True)

        # Grip and
        rob.grip(closed=gripper['closed'], force=gripper['force'])

        # Go back up
        _move_to_height(rob, travel_height, vel, a, movelinear=True)

        # Check that the part is gripped using grip actual value
        if abs(rob.gripper_actual_pos() - gripper['closed']) < 1:
            # Not gripped anything. Open and try again.
            rob.grip(closed=gripper['open'])
            continue

        # Get target place pose
        target_pose = _build_drop_off_pose(rob, platf_calib['taught_poses']['build'],
                           target, unit_brick_dims)

        # Start moving in another thread
        t = Thread(target=_move_to_height, args=(rob, travel_height, vel, a, target_pose))
        t.daemon = True
        t.start()

        # Check twice that the part is gripped using MV
        detection_failures = 0
        if failure_detection_on:
            for i in range(2):
                if not mv.color_in_region(target[4], finger_region, min_area=0.15):
                    detection_failures += 1

        if detection_failures > 1:  # redo previous brick
            t.join(timeout=10)  # join move thread
            continue
        else:  # Place brick and advance to next brick
            t.join(timeout=10)  # join move thread

            # Place
            # move just above brick
            platf_heigth = platf_calib['taught_poses']['build'][0][2]
            target_z = platf_heigth + (target[0] - 1) * unit_brick_dims['base_height']
            z = target_z + unit_brick_dims['base_height'] * 1.05
            _move_to_height(rob, z, vel, a, target_pose, movelinear=True)

            # Move slower to place
            _move_to_height(rob, target_z, vel / 2, a / 3, target_pose, movelinear=True)

            rob.grip(closed=gripper['open'])

            step_idx += 1
            # Save state
            state['current_index'] = step_idx
            with open(state_fname, 'w') as f:
                f.write(json.dumps(state))

            _move_to_height(rob, travel_height, vel, a, movelinear=True)


def deconstruct(rob, gripper, tcp, platf_calib, plan, travel_height, unit_brick_dims):
    """ Deconstruct built model.

    Parts are dropped to the middle of the imaging area at travel height + 10cm.
    The robot arm wiggles the pieces in order to loosen them before taking them off.
    Due to low friction the robot might need to try multiple times before it succeeds.

    Multiple parts may be detached at once. Its okay as the program will later notice that
    some pieces are already gone.

    """

    vel = 1.5
    a = 1.2

    rob.popup('Continue to start deconstructing', blocking=True)

    rob.set_tcp(tcp['gripper'])
    # Drop to middle of imaging area
    drop_off_pose = _midpoint_of_poses(platf_calib['taught_poses']['part'][0],
                                      platf_calib['taught_poses']['part'][1])
    drop_off_pose[2] = drop_off_pose[2] + travel_height + 10 / 100

    plan = deepcopy(plan)
    plan.reverse()
    rob.grip(closed=gripper['open'])

    for piece in plan:
        pose = _build_drop_off_pose(rob, platf_calib['taught_poses']['build'], piece, unit_brick_dims)

        # _build_drop_off_pose doesn't compute the height so its done below
        platf_heigth = platf_calib['taught_poses']['build'][0][2]
        target_z = platf_heigth + (piece[0] - 1) * unit_brick_dims['base_height']
        pose[2] = target_z

        _move_to_height(rob, pose[2] + 0.04, vel, a, pose, movelinear=False)
        rob.movel(pose, v=vel, a=a/2)
        actual_grip_amt = rob.grip(closed=gripper['closed'], force=gripper['force'])

        # Check that the part is gripped using grip actual value
        if abs(actual_grip_amt - gripper['closed']) < 1:
            # Not gripped anything. Move to next piece.
            rob.grip(closed=gripper['open'])
            continue

        else:

            _detach_brick(rob)
            while abs(rob.gripper_actual_pos() - gripper['closed']) < 1:
                # Not gripped anything. Try again
                rob.grip(closed=gripper['open'])
                rob.movel(pose, v=vel, a=a)
                rob.grip(closed=gripper['closed'], force=gripper['force'])
                _detach_brick(rob, up_move=2)

            # Drop off
            _move_to_height(rob, drop_off_pose[2], vel=vel, a=a, movelinear=False)
            rob.movej(drop_off_pose, v=vel, a=a)
            rob.grip(closed=gripper['open'])
            _move_to_height(rob, drop_off_pose[2], vel=vel, a=a, pose=pose ,movelinear=False)

    print('Deconstruction done!')


def _build_drop_off_pose(rob, build_platf, target, unit_brick_dims):
    """ Compute pose in which the brick is to be placed.
    """

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

    return pose


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
    """ Using points divide the rectangle's x and y to even sized steps so that
    each step is as close as possible to the provded 'ideal' value.

    Aim is to spread the positioning error evenly across the whole rectangle.

    X and Y axis and origin are decided using right hand rule.

    Returns
    -------
    float, float
        x and y.
    """

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
    list
        Pose with angle chosen according to 'preferred_angle'.

    """

    rpy = rob.rotvec2rpy(pose[3:])
    if abs(rpy[2] - preferred_angle) > radians(90):
        rpy[2] = rpy[2] + radians(180)
        pose = deepcopy(pose)
        pose[3:] = rob.rpy2rotvec(rpy)
    return pose


def _find_brick_iteratively(rob, mv, target, tcp, imaging_area, travel_height,
                            vel, a, max_diff=0.7, allowed_rect_area=None, use_color_vision=True, use_model=True):
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

            if use_color_vision:
                target_color = target[4]
            else:
                target_color = None
            match = mv.find_brick(target_color, size, margin=0.2,
                                  use_max_edge=use_max_edge, draw=True, use_model=use_model)
        except NoMatches:
            random_pose = deepcopy(pose)
            rand_xy = _random_point_within_rect(allowed_rect_area[0], allowed_rect_area[1])
            random_pose[0:2] = rand_xy
            rob.movej(random_pose, v=vel, a=a)
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
                # Go to random point within imaging area
                random_pose = deepcopy(pose)
                rand_xy = _random_point_within_rect(allowed_rect_area[0], allowed_rect_area[1])
                random_pose[0:2] = rand_xy
                rob.movej(random_pose, v=vel, a=a)
                continue

        rob.movej(pose_, v=vel, a=a, relative=True)

    # Move gripper where the camera is
    pose = rob.get_tcp()
    rob.set_tcp(tcp['gripper'])
    rob.movej(pose, v=vel, a=a)

    return

def _detach_brick(rob, up_move=2):
    """ Detach brick from build platform by wiggling it.

    Parameters
    ----------
    rob
    up_move : float
        How much up movement is allowed for the wiggling (mm)

    Returns
    -------

    """
    pose = rob.get_tcp()
    vel = 0.2
    acc = 0.2
    roll = 0.07
    up_move /= -1000

    roll_poses = []
    roll_poses.append(rob.pose_trans(pose, [0, 0, up_move] + rob.rpy2rotvec([0, -roll, 0])))
    roll_poses.append(rob.pose_trans(pose, [0, 0, up_move] + rob.rpy2rotvec([-roll, 0, 0])))
    roll_poses.append(rob.pose_trans(pose, [0, 0, up_move] + rob.rpy2rotvec([0, roll, 0])))
    roll_poses.append(rob.pose_trans(pose, [0, 0, up_move] + rob.rpy2rotvec([roll, 0, 0])))

    # Randomize wiggling order
    if np.random.randn(1)[0] > 0.5:
        roll_poses.reverse()

    for p in roll_poses:
        rob.movej(p, v=vel, a=acc)

    pose_ = rob.pose_trans(pose, [0, 0, -0.05] + [0, 0, 0])
    rob.movel(pose_, v=vel, a=acc)


def _random_point_within_rect(p1, p2):
    # For a rectangle defined by two points p1 and p2.
    rn_x = np.random.uniform(min(p1[0], p2[0]), max(p1[0], p2[0]), size=1)[0]
    rn_y = np.random.uniform(min(p1[1], p2[1]), max(p1[1], p2[1]), size=1)[0]
    return [rn_x, rn_y]