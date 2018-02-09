import trollius
from trollius import From

import pygazebo
from pygazebo.msg import poses_stamped_pb2

import numpy as np
import matplotlib.pyplot as plt

import os
import sys
import subprocess
import time
import pty
import serial
import re

master, slave = pty.openpty()
virtual_port_process = subprocess.Popen(
        ['socat', '-d', '-d', 'pty,raw,echo=0', 'pty,raw,echo=0'],
        shell=False,
        stdout=slave,
        stderr=slave,
        stdin=subprocess.PIPE)

stdout = os.fdopen(master)

simulator_input_port = re.sub('\r', '', stdout.readline().split('\n')[0].split(" ")[-1])
simulator_output_port = re.sub('\r', '', stdout.readline().split('\n')[0].split(" ")[-1])

uwb_tag = serial.Serial(simulator_input_port, baudrate=115200, write_timeout=0.02)

ANCHOR_0 = [-4.00, 3.00, 1.00]
ANCHOR_1 = [4.00, 3.00, 1.00]
ANCHOR_2 = [4.00, -3.00, 1.00]
ANCHOR_3 = [-4.00, -3.00, 1.50]

sys.stdout.write("UWB Tag Simulator Stated ... \n")
stdout_title = '\n'.join(
        [
            "Dev: Use \033[1;32;40m{0}\033[0m to receive uwb tag data".format(simulator_output_port),
            "Quick Test: cat <{0}".format(simulator_output_port),
            "Press Ctrl+C to Quit \n"

        ])
sys.stdout.write(stdout_title)

from threading import Thread

import struct
def to_hex(f):
    return format(int(f * 1000), 'x').zfill(8)

from filterpy.stats import gaussian
import random

lock = False

def callback(data):
    try:
        message = poses_stamped_pb2.PosesStamped.FromString(data)
        pose = message.pose[0].position
        r0 = np.sqrt(np.square(pose.x - ANCHOR_0[0]) + np.square(pose.y - ANCHOR_0[1]) + np.square(pose.z - ANCHOR_0[2]))   
        r1 = np.sqrt(np.square(pose.x - ANCHOR_1[0]) + np.square(pose.y - ANCHOR_1[1]) + np.square(pose.z - ANCHOR_1[2]))   
        r2 = np.sqrt(np.square(pose.x - ANCHOR_2[0]) + np.square(pose.y - ANCHOR_2[1]) + np.square(pose.z - ANCHOR_2[2]))   
        r3 = np.sqrt(np.square(pose.x - ANCHOR_3[0]) + np.square(pose.y - ANCHOR_3[1]) + np.square(pose.z - ANCHOR_3[2]))   

        global lock
        global uwb_tag
        if lock == False:
            lock = True
            sys.stdout.write('\rx: %f, y: %f, z: %f, r0: %f, r1: %f, r2: %f, r3: %f'%(pose.x, pose.y, pose.z, r0, r1, r2, r3))
            sys.stdout.flush()

            #  print('mr 0f {0} {1} {2} {3} 0958 c0 40424042 a0:0\n'.format(to_hex(r0), to_hex(r1), to_hex(r2), to_hex(r3)).encode())
            try:
                uwb_tag.write('mr 0f {0} {1} {2} {3} 0958 c0 40424042 a0:0\n'.format(to_hex(r0), to_hex(r1), to_hex(r2), to_hex(r3)).encode('utf-8'))
            except serial.SerialTimeoutException:
                uwb_tag.flushOutput()

            time.sleep(0.01)
            lock = False
    except Exception as e:
        print(e)
        raise e

def subscribe_loop():
    manager = yield From(pygazebo.connect(address=('192.168.1.49', 11345)))

    manager.subscribe('/gazebo/default/pose/info',
                      'gazebo.msgs.PosesStamped',
                      callback)

    while True:
        yield From(trollius.sleep(1))

loop = trollius.get_event_loop()
trollius.set_event_loop(loop)
loop.run_until_complete(subscribe_loop())
