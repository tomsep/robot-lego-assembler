from __future__ import division, print_function
import time

from math import radians
from pymodbus.constants import Endian
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder

from legoassembler.communication import URClient


class Robot:

    def __init__(self, ip, grip_preamble, grip_funcdef, no_gripper=False):

        self.mod_client = ModbusTcpClient(ip, 502)
        self.script_client = URClient()
        self.script_client.connect(ip, 30001)
        self._state_reg = 128
        self._no_gripper = no_gripper

        if self._no_gripper is False:
            self._load_gripper_script(grip_preamble, grip_funcdef)


    def move(self, mtype, pose, v, a, relative=False):

        # TODO: add defaults for v,a (different for all move types)
        if relative:
            prog = 'pose = get_actual_tcp_pose()' \
                   'pose = pose_add(pose, p{})' \
                   'move{}(pose,a={},v={})' \
                   ''.format(pose, mtype, a, v)
        else:
            prog = 'move{}(p{},a={},v={})'.format(mtype, pose, a, v)

        self._run(prog)

    def move_joints(self, mtype, pose, v, a, relative=False):

        if relative:
            prog = 'p1 = get_actual_tcp_pose()' \
                   'p2 = p{}' \
                   'pose = pose_add(p1, p2)' \
                   'move{}(pose,a={},v={})' \
                   ''.format(pose, mtype, a, v)
        else:
            prog = 'move{}({},a={},v={})'.format(mtype, pose, a, v)

        self._run(prog)

    def teachmode(self, on, popup=''):
        if on:
            prog = 'teach_mode()\n'
            if popup != '':
                prog += 'popup("{}", blocking=True)\n'.format(popup)

        else:
            prog = 'end_teach_mode()\n'

        self._run(prog)

    def get_tcp(self):
        # TODO: use decimal to get rid of floating point errors?
        regs = self.mod_client.read_holding_registers(400, 6).registers
        xyz = [x / 10000 for x in regs[:3]]

        for i in range(len(xyz)):
            if xyz[i] > 3.2768:
                xyz[i] = -(6.5535 - xyz[i])

        rxyz = [x / 1000 for x in regs[3:]]
        for i in range(len(rxyz)):
            if rxyz[i] > 32.768:
                rxyz[i] = -(65.535 - rxyz[i])
            elif rxyz[i] > radians(360):
                rxyz[i] -= radians(360)
        pose = xyz + rxyz
        return pose # meters, and rads

    def get_joint_positions(self):

        # TODO: use decimal to get rid of floating point errors?

        joints = self.mod_client.read_holding_registers(270, 6).registers
        joints = [x / 1000 for x in joints]

        # Write signs prog
        prog = """
def fun(addr, val):
    if val >= 0:
        write_port_register(addr, 1)
    else:
        write_port_register(addr, 0)
    end
end
ps = get_actual_joint_positions()
fun(130, ps[0])
fun(131, ps[1])
fun(132, ps[2])
fun(133, ps[3])
fun(134, ps[4])
fun(135, ps[5])

"""
        self._run(prog)
        signs = self.mod_client.read_holding_registers(130, 6).registers

        joints_ = []
        for val, sign in zip(joints, signs):
            if int(sign) == 1:
                joints_.append(val)
            else:
                joints_.append(-(radians(360) - val))

        return joints_  # rads

    def popup(self, msg, blocking=False):
        prog = 'popup("{}", blocking={})\n'.format(msg, blocking)
        self._run(prog)

    def _run(self, prog):
        self.mod_client.write_register(128, False)

        #prog = prog.replace('\t', '    ')  # tabs to 4 spaces
        prog += '\nwrite_port_register({}, 1)\n'.format(self._state_reg)
        # Add 4 spaces to each line
        prog = (4 * ' ').join(('\n' + prog.lstrip()).splitlines(True))

        script = 'def program():\n{}\nend\n'\
            .format(prog)


        self.script_client.send(script)
        while not self.mod_client.read_holding_registers(128, 1).registers[0]:
            if self.operable() is not True:
                raise RuntimeError('Robot not operable')
            time.sleep(0.02)

    def operable(self):
        status = self.mod_client.read_coils(260, 1)
        # TODO: add emergency states and such
        return status.bits[0]

    def grip(self, closed, speed=10, force=10):

        if self._no_gripper:
            return

        s = self._gripper_script.replace('$$CLOSED$$', str(closed))
        s = s.replace('$$SPEED$$', str(speed))
        s = s.replace('$$FORCE$$', str(force))
        self._run(s)

    def _load_gripper_script(self, preamble, funcdef):

        with open(preamble, 'r') as f:
            script = f.read()

        with open(funcdef, 'r') as f:
            fdef = f.read()

        script += '\n' + fdef + '\n'
        self._gripper_script = script

if __name__ == '__main__':

    ip = '192.168.71.128'
    ip = '192.168.1.198'
    rob = Robot(ip, 's', 's', no_gripper=True)

    pose = [0, 0, 0, 0, 0, radians(15)]
    rob.move_joints('j', pose, v=0.2, a=0.1, relative=True)
    print('done')