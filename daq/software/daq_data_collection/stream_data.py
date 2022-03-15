#!/usr/bin/env python3
"""for collecting the analog data as it arrives."""

import sys, signal
import csv
import usb.core
import struct
import time
import argparse

import numpy as np
from gr.pygr import mlab


# Summary:
# Unpack the data from the transferred USB Packets. Alert on dropped packets.
# Store data in a FIFO. Plot it periodically.


# Plotting/Data Collection Constants
USB_READ_TIMEOUT_MS = 3000 # How long (in ms) to wait until giving up on incoming data.
PLOTTING_PTS = 5000

# USB Device-related constants. These must agree with the settings on the device.
USB_ID_VENDOR = 0xcafe
USB_ID_PRODUCT = 0xcafe
BULK_READ_ADDR = 0x81 # bulk transfer address.
BULK_PKT_SIZE = 64 # size of the bulk transfer packet in bytes.
SAMPLING_FREQ_HZ = 1000. # sampling rate in samples/sec.
DATAPTS_PER_PACKET = 30
NUM_CHANNELS = 1
USB_PACKET_DELIVERY_FREQ_HZ = SAMPLING_FREQ_HZ/DATAPTS_PER_PACKET # approx usb packet delivery rate (samples/sec).

# Current measurement range for Pololu PN 4033. Change if using a different sensor.
MAX_AMPS = 40
MIN_AMPS = -40

MAX_VOLTS = 3.205 # Voltage Divider max output
VOLTAGE_FULL_SCALE_RANGE = 3.3
VOLTS_TO_BITS = ((1 << 12) - 1) / VOLTAGE_FULL_SCALE_RANGE

MAX_BITS = MAX_VOLTS * VOLTS_TO_BITS # ~994 bits full-scale
MIN_BITS = 0
# Conversion
BITS_TO_AMPS_SLOPE = (MAX_AMPS - MIN_AMPS)/(MAX_BITS - MIN_BITS)
BITS_TO_AMPS_Y_INTERCEPT = MIN_AMPS


def bits_to_amps(bits):
    """helper function to convert raw data from usb device to amps"""
    return bits*BITS_TO_AMPS_SLOPE + BITS_TO_AMPS_Y_INTERCEPT


def stream_data_loop(usb_daq):
    """Run usb data collection for specified number of seconds.
    Attributes:
        recording_time_s: the desired recording time in seconds.
        usb_daq: the device to record data from.
    """

    # Data collection plotting.
    mlab.title("Current [A] vs time [s]")
    mlab.xlabel("time [sec]")
    mlab.ylabel("current [A]")

    # Data collection setup.
    agg_measurements = np.zeros(PLOTTING_PTS) # container for aggregate measurements

    last_packet_index, new_data = collect_usb_packet(usb_daq)
    # Store first measurement in the FIFO.
    agg_measurements = np.roll(agg_measurements, -len(new_data))
    agg_measurements[-len(new_data):] = new_data

    # Loop forever.
    while True:
        # Collect new data.
        packet_index, new_data = collect_usb_packet(usb_daq)
        print(f"Receiving 64-byte packet: {packet_index}", end="\r", flush=True)
        # If we've read this packet before, skip it.
        if (packet_index == last_packet_index):
            continue

        # If we've missed packets, print an error and stuff missing data with zeros.
        packet_delta = packet_index + ~last_packet_index + 1 # 2's comp subtraction to handle rollover.
        missing_packets = packet_delta - 1
        if missing_packets > 0:
            print(f"Error: we have dropped {missing_packets} packets(s)! \
                    Sampling rates is too fast.")
            # Stuff with zeros so that we can still look at the data.
            for missing_packet in range(missing_packets):
                agg_measurements = np.roll(agg_measurements, -len(new_data))
                agg_measurements[-len(new_data):] = ([0]*DATAPTS_PER_PACKET)
            # Continue to default case where we will store the data from this packet.

        # Default case:
        agg_measurements = np.roll(agg_measurements, -len(new_data))
        agg_measurements[-len(new_data):] = new_data
        last_packet_index = packet_index

        # plotting!
        mlab.plot(range(len(agg_measurements)), agg_measurements,
             xlim=(0, PLOTTING_PTS), ylim=(MIN_AMPS, MAX_AMPS))

    print()
    return agg_measurements


def collect_usb_packet(usb_daq):
    """ return the packet index and the measurements of a single packet.

        Measurements have been converted out of array of bytes into
        array of unsigned ints. Data is unscaled.
    """
    # analog data in the usb packet is formatted as:
    # 1 contiguous chunk of 30 little-endian, 16-bit unsigned values.
    # 1 uint32_t packet index. Increments per packet

    # Each packet has an index. We will treat it as a fixed int so we can do
    # subtraction to figure out if the packet that we just read is new.
    analog_packet_data = [] # storage container

    # Pull a packet from the usb device.
    # TODO: try-except here to catch when we're not getting data fast enough.
    usb_pkt = usb_daq.read(BULK_READ_ADDR, BULK_PKT_SIZE, timeout=USB_READ_TIMEOUT_MS)
    # Extract data from the packet and put in the respective queues.
    pkt_index = struct.unpack("<L", usb_pkt[BULK_PKT_SIZE-4:])[0]
    # Unpacking nonstandard (3-byte) values is a bit harder.
    data_bytes = usb_pkt[:-4] # first 60 bytes.
    data_fmt = f"<{DATAPTS_PER_PACKET}H" # unsigned short.
    analog_packet_data = struct.unpack(data_fmt, data_bytes)

    return pkt_index, [bits_to_amps(sample) for sample in analog_packet_data]


if __name__ == "__main__":
    parser  = argparse.ArgumentParser()
    parser.add_argument("--simulated", type=bool, default=False)

    args = parser.parse_args()

    usb_daq = None
    if args.simulated:

        from fake_usb_daq import USBDaqStub

        print("spoofing usb connection. Sending fake data.")
        usb_daq = USBDaqStub()
    else: # actually connect to real USB hardware.
        print("Connecting to USB DAQ...", end="")
        try:
            usb_daq = usb.core.find(idVendor=USB_ID_VENDOR, idProduct=USB_ID_PRODUCT)
            if usb_daq is None:
                print("Failed.")
                raise ValueError('Device not found. Is it plugged in?')
            usb_daq.set_configuration()
            print("Done.")
        except usb.core.USBError as e:
            print("Failed.")
            print(e)
            print("Error: this program must be run with root, or you must apply udev rules.")
            sys.exit()

    # Setup a graceful CTRL-C catch.
    signal.signal(signal.SIGINT, lambda :sys.exit(1))
    stream_data_loop(usb_daq)

