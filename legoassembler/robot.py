from __future__ import division, print_function
import time

from math import radians
from pymodbus.constants import Endian
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder

from legoassembler.communication import URClient


class Robot:

    def __init__(self, ip):

        self.mod_client = ModbusTcpClient(ip, 502)
        self.script_client = URClient()
        self.script_client.connect(ip, 30001)
        self._state_reg = 128

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
            prog = 'p1 = get_actual_joint_positions()' \
                   'p2 = p{}' \
                   'p1[0] = p1[0] + p2[0]' \
                   'p1[1] = p1[1] + p2[1]' \
                   'p1[2] = p1[2] + p2[2]' \
                   'p1[3] = p1[3] + p2[3]' \
                   'p1[4] = p1[4] + p2[4]' \
                   'p1[5] = p1[5] + p2[5]' \
                   'pose = get_forward_kin(p1)' \
                   'move{}(pose,a={},v={})' \
                   ''.format(pose, mtype, a, v)
        else:
            prog = 'move{}({},a={},v={})'.format(mtype, pose, a, v)

        self._run(prog)

    def teachmode(self, on):
        if on:
            prog = 'teach_mode()\n'
        else:
            prog = 'end_teach_mode()\n'

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
        pass

if __name__ == '__main__':

    ip = '192.168.71.128'
    rob = Robot(ip)
    print(rob.get_joint_positions())
    builder = BinaryPayloadBuilder(byteorder=Endian.Big,
                                   wordorder=Endian.Little)
    builder.add_16bit_int(-155)
    payload = builder.to_registers()
    print(payload)
    #payload = builder.build()
    #print(payload)
    #rob.mod_client.write_registers(0, payload, unit=1)
    regs = rob.mod_client.read_holding_registers(0, 1, skip_decode=True, skip_encode=True)
    #decoder = BinaryPayloadDecoder.fromRegisters(regs.registers, byteorder=Endian.Big, wordorder=Endian.Big)
    #print(decoder.decode_32bit_float())

    print(regs.registers)