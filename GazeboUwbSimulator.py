import trollius
from trollius import From

import time

import pygazebo
from pygazebo.msg import poses_stamped_pb2

import struct

import numpy as np

import serial

MSG_HEADER =           0x01
MSGID_BEACON_CONFIG =  0x02
MSGID_BEACON_DIST =    0x03
MSGID_POSITION =       0x04

fc = serial.Serial('/dev/cu.SLAB_USBtoUART', baudrate=115200, timeout=0.01)

ANCHORS = [
            [0.00, 0.00, 1.00],
            [6.500, 0.00, 1.00],
            [0.00, -3.50, 1.00],
            [6.500, -3.50, 1.50]
        ]

def send_msg(msg_id, data):
    msg_len = len(data) + 1
    checksum = 0
    checksum ^= msg_id
    checksum ^= msg_len
    for i in struct.unpack("<{0}B".format(len(data)), data):
        checksum = checksum ^ i

    fc.write(struct.pack('<B', MSG_HEADER))
    fc.write(struct.pack('<B', msg_id))
    fc.write(struct.pack('<B', msg_len))
    fc.write(data)
    fc.write(struct.pack('<B', checksum))

def callback(data):
    message = poses_stamped_pb2.PosesStamped.FromString(data)
    pose = message.pose[0].position

    print(time.time())
    try:
        #  print(pose)
        send_msg(MSGID_POSITION, struct.pack("<iiih", int(pose.x*1000), int(pose.y*1000), int(pose.z*1000), int(0.01*1000)))

        for i in range(4):
            beacon_id = i
            anchor = ANCHORS[i]
            send_msg(MSGID_BEACON_CONFIG, struct.pack("<BBiii", beacon_id, 4, int(anchor[0]*1000), int(anchor[1]*1000), int(anchor[2]*1000)))
            r = np.linalg.norm(np.array([pose.x, pose.y, pose.z]) - np.array(anchor))  
            #  print(i, r)
            send_msg(MSGID_BEACON_DIST, struct.pack("<Bi", beacon_id, int(r*1000)))

        time.sleep(0.2)
    except Exception as e:
        print(e)


def subscribe_loop():
    manager = yield From(pygazebo.connect(address=('10.211.55.5', 11345)))

    manager.subscribe('/gazebo/default/pose/info',
                      'gazebo.msgs.PosesStamped',
                      callback)

    while True:
        yield From(trollius.sleep(1))

loop = trollius.get_event_loop()
trollius.set_event_loop(loop)
loop.run_until_complete(subscribe_loop())
