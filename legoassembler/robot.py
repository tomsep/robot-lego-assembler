from __future__ import division, print_function
import json
from pymodbus.client.sync import ModbusTcpClient
from copy import deepcopy

from legoassembler.communication import URClient, URServer


class Robot:
    """ Robot operating class for UR5

    Implements methods for operating Universal Robot 5
    with optional Robotiq gripper.
    Also makes possible to retrieve status and values from the UR5 controller.

    Uses ModBus and Sockets for communication.

    Each command is sent as a script:
    def program():
        socket_open(<ip>, <port>)
        <code>
        socket_send_line(<some value>)
    end
    which always returns a line to the listening socket. This return value
    (sometimes empty) indicates when the script sent has finished and the next
    script can be safely sent.

    Most methods are named similarly as in the UR script manual. See the documentation
    for details of these methods.

    """

    def __init__(self, ip_ur, ip_host, grip_def=None, port_host=26532):

        self.mod_client = ModbusTcpClient(ip_ur, 502)
        self._script_client = URClient()
        self._script_client.connect(ip_ur, 30001)
        self._receiver = URServer(ip_host, port_host)
        self._ip_host = ip_host
        self._port_host = port_host

        if grip_def:
            with open(grip_def, 'r') as f:
                self._grip_def = f.readlines()
        else:
            self._grip_def = ['']
    def movel(self, pose, a=1.2, v=0.25, relative=False):
        if relative:
            prog = ['pose = get_actual_tcp_pose()',
                   'pose = pose_add(pose, p{})'.format(pose),
                   'movel(pose,a={},v={})'.format(a, v)]
        else:
            prog = ['movel(p{},a={},v={})'.format(pose, a, v)]

        prog += ['socket_send_line("")']
        self._run(prog)

    def movej(self, pose, a=1.4, v=1.05, relative=False):
        if relative:
            prog = ['pose = get_actual_tcp_pose()',
                   'pose = pose_add(pose, p{})'.format(pose),
                   'movej(pose,a={},v={})'.format(a, v)]
        else:
            prog = ['movej(p{},a={},v={})'.format(pose, a, v)]

        prog += ['socket_send_line("")']
        self._run(prog)

    def teachmode(self, msg):
        prog = ['teach_mode()',
                'popup("{}", blocking=True)'.format(msg),
                'socket_send_line("")']
        self._run(prog)

    def get_tcp(self):
        prog = \
            ['ps = get_actual_tcp_pose()',
             'socket_send_line(ps)']
        return json.loads(self._run(prog)[1:])

    def get_joint_positions(self):
        prog = \
            ['ps = get_actual_joint_positions()',
             'socket_send_line(ps)']
        return json.loads(self._run(prog))

    def popup(self, msg, blocking=False):
        prog = \
            ['popup("{}", blocking={})\n'.format(msg, blocking),
             'socket_send_line("")']
        self._run(prog)

    def rpy2rotvec(self, rpy_vec):
        prog = \
            ['rpy = rpy2rotvec({})'.format(rpy_vec),
             'socket_send_line(rpy)']
        return json.loads(self._run(prog))

    def rotvec2rpy(self, rot_vec):
        prog = \
            ['rotvec = rotvec2rpy({})'.format(rot_vec),
             'socket_send_line(rotvec)']
        return json.loads(self._run(prog))

    def grip(self, closed, speed=10, force=10):
        if self._grip_def:
            prog = deepcopy(self._grip_def)
            for i in range(len(self._grip_def)):
                prog[i] = prog[i].replace('$$CLOSED$$', str(closed))
                prog[i] = prog[i].replace('$$SPEED$$', str(speed))
                prog[i] = prog[i].replace('$$FORCE$$', str(force))

            self._run(prog + ['socket_send_line("")'])
        else:
            raise ValueError('No gripper definition script defined.')

    def pose_trans(self, p_from, p_from_to):
        """ Transform pose using another pose

        Returns
        -------
        list[float, ..] length 6

        """
        prog = \
            ['ps = pose_trans(p{},p{})'.format(p_from, p_from_to),
             'socket_send_line(ps)']
        return json.loads(self._run(prog)[1:])

    def set_tcp(self, pose):
        prog = \
            ['set_tcp(p{})'.format(pose),
             'socket_send_line("")']
        self._run(prog)

    def force_mode_tool_z(self, force, time):
        prog = \
            ['task_frame = tool_pose()',
             'sel_vector = [0,0,1,0,0,0]',
             'type = 2',
             'wrench = [0,0,{},0,0,0]'.format(force),
             'limits = [0.1, 0.1, 0.15, 0.3490658503988659, 0.3490658503988659, 0.3490658503988659]',
             'force_mode(task_frame, sel_vector, wrench, type, limits)'
             'sleep({})'.format(time),
             'end_force_mode()',
             'stopl(5.0)',
             'socket_send_line("")']
        self._run(prog)

    def _run(self, sub_prog):

        sub_prog = ['\t' + x for x in sub_prog]  # add tabs to sub program
        script = \
            ['def prg():',
             '\tsocket_open("{}",{})'.format(self._ip_host, self._port_host)] +\
            sub_prog + ['end']

        script = '\n'.join(script) + '\n'

        self._script_client.send(script)
        self._receiver.accept(print_info=False)
        data = self._receiver.recv()
        self._receiver.close()
        return data
