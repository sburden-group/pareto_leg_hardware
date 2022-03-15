#!/usr/bin/env python3
import struct
import random


class USBDaqStub(object):

    def __init__(self):
        """Constructor."""
        self.sent_packets = 0


    def read(self, *args, **kwargs):
        """Create fake data (all zeros), but increment packet on each read."""
        #a0_bytes = 30*[0]
        a0_bytes = [int(random.gauss((1 << 12)/2, 1<<8)) for i in range(30)]
        raw_packet = struct.pack("<30H1L", *a0_bytes, self.sent_packets)
        self.sent_packets += 1 # increment the packet.
        return raw_packet


if __name__ == "__main__":
    my_daq = USBDaqStub()
    print(my_daq.read())
    print(my_daq.read())
    print(my_daq.read())
