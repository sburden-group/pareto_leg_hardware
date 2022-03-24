#!/usr/bin/env python3
"""Demo for reading robot motor angles."""

from pareto_leg.odrive_driver import OdriveDriver
import odrive

import time, sys
import numpy as np
from numpy import pi

## Calibration Constants for such that the controller and the real-world
## joint angles match.
CALIB_POSITION = np.asarray([pi/2, -pi/2]) # [rad]. Calibration "stance"
CALIB_MEASUREMENT = np.asarray([.110, -4.50])   # Measured real-world angles
                                                # when the robot is in the
                                                # calibration "stance".

# Hardware Connection Setup:
print("Connecting to ODrive... ")
odrv0 = odrive.find_any()
my_odd = OdriveDriver(odrv0)

## Constants
LOOP_TIME_S = 0.1 # [sec]. How long between sending new data to the ODrive
curr_time_s = time.perf_counter()
prev_time_s = curr_time_s
while True:
    curr_time_s = time.perf_counter()
    if (curr_time_s - prev_time_s) > LOOP_TIME_S:
#        # Get Measurements: flip sign on motor 0 to match world config.
        theta1, theta2 = np.array(my_odd.get_motor_angles()) * np.array([1, -1]) # flip sign
        print(f"rawtheta1: {theta1:.3f} | rawtheta2: {theta2:.3f} | ", end="")
        theta1, theta2 = [theta1, theta2] + CALIB_POSITION - CALIB_MEASUREMENT
        #theta1 = theta1 % (np.pi) * np.sign(theta1)
        # theta2 = theta2 % (np.pi) * np.sign(theta2)
        print(f"theta1: {theta1:.3f} | theta2: {theta2:.3f}")
#        # Setup next loop interation.
        prev_time_s = curr_time_s

