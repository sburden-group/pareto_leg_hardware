#!/usr/bin/env python3
"""Driver for a pd controller"""

from pareto_leg.odrive_driver import OdriveDriver
import odrive

import time
import numpy as np
from numpy import pi

CONTROL_LOOP_TIME_S = 0.01 # seconds (update rate of 10 Hz)

class PDController(object):

    def __init__(self):
        self.Kp = 180/6.28/100 * 6
        self.Kd = self.Kp/4


    def update(self, q, qdot):
        """Perform control loop update."""
        error = [pi/8-q[0],
                -pi/8-q[1]]
        error_dot = [-qdot[0],
                     -qdot[1]]

        return np.array([self.Kp*error[0]+self.Kd*error_dot[0],
                         self.Kp*error[1]+self.Kd*error_dot[1]])


if __name__ == "__main__":

    CALIB_MEASUREMENT = np.asarray([-1.38, 3.35])
    CALIB_POSITION = np.asarray([pi/2, -pi/2])

    print("Connecting to ODrive.")
    odrv0 = odrive.find_any()
    my_odd = OdriveDriver(odrv0)
    my_odd.set_torque_control_mode()
    my_odd.arm()

    print("Connected.")
    my_controller = PDController()

    curr_time_s = time.perf_counter()
    prev_time_s = curr_time_s

    while True:

        curr_time_s = time.perf_counter()
        if (curr_time_s - prev_time_s) > CONTROL_LOOP_TIME_S:
            # Get Measurements: flip sign on motor 0 to match world configuration. (see pic)
            thetas = np.array(my_odd.get_motor_angles()) * np.array([-1, 1]) # flip sign

            q = thetas + CALIB_POSITION - CALIB_MEASUREMENT

            # Clamp thetas to within (-2pi, +2pi) range
            for i, _ in enumerate(thetas):
                while thetas[i] < -2*pi:
                    thetas[i] += 4*pi
                while thetas[i] > 2*pi:
                    thetas[i] -= 4*pi

            qdot = np.array(my_odd.get_motor_velocities()) * np.array([-1, 1]) # flip sign.

            #print(f"q is:    ({q[0]:.3f}, {q[1]:.3f})", end=" ")
            #print(f"qdot is: ({qdot[0]:.3f}, {qdot[0]:.3f})")

            # Update Control Loop
            current_a = my_controller.update(q, qdot)
            print(f"({current_a[0]:.4f}, {current_a[1]:.4f})")
            tf_current_a = current_a * np.array([-1, 1])
            my_odd.set_torques(*tf_current_a)
            # Setup for next loop interation.
            prev_time_s = curr_time_s

