#!/usr/bin/env python-jl
"""Demo for connecting to Julia."""

from pareto_leg.odrive_driver import OdriveDriver
import odrive

import time, sys
import numpy as np
from numpy import pi

import julia
from julia.core import UnsupportedPythonError
# Setup Julia package with Hopper/Handshake Dynamics
try:
    from julia import Pkg, Main
except UnsupportedPythonError as e:
    print("Error: this script must be un with python-jl interpreter.")
    sys.exit()

print("Setting up Julia pareto-leg-control package... ")
Pkg.activate("../pareto-leg-control/LegControllers") # This string points to the pareto-leg-control folder.

# bind included modules to variables:
from julia import LegControllers
model = LegControllers.Model
designs = LegControllers.Designs
controller = LegControllers.ComputedTorque

# design parameters for a sample robot.
# design parameters, see Designs.jl for description of the arguments
leg_params = designs.Params(
    .1,         # Ext Spring 1 free length
    0.,       # Ext Spring 1 Spring Constant
    0.0,        # Ext Spring 1 Initial Tension
    3.14/4,     # Ext Spring 1 Rest angle
    .1,         # Ext Spring 2 Free Length
    0.,       # Ext Spring 2 Spring Constant
    0.0,        # Ext Spring 2 Initial Tension
    -3.14/4,    # Ext Spring 2 Rest Angle
    .27,        # Comp Spring Free Length
    0.,       # Comp Spring Spring Constant
    .07,        # Femur Length
    .2          # Tibya Length
)

# Connect to ODrive.
# Get Starting Position

# Hardware Connection Setup:
print("Connecting to ODrive... ")
odrv0 = odrive.find_any(timeout=1)
my_odd = OdriveDriver(odrv0)
my_odd.set_torque_control_mode()
#my_odd.arm()

## Calibration Constants for such that the controller and the real-world
## joint angles match.
CALIB_POSITION = np.asarray([pi/2, -pi/2]) # [rad]. Calibration "stance"
CALIB_MEASUREMENT = np.asarray([-1.316, -2.99])   # Measured real-world angles
                                                  # when the robot is in the
                                                  # calibration "stance".


def get_state():
    """returns the state vector by querying the ODrive for required data."""

    # Note: vector q = [x_body, # fixed at 0.
    #                   motor_theta1,
    #                   motor_theta2,
    #                   x_foot,
    #                   y_foot,
    # Note: vector qdot is just the time derivative of q.

    # First compute q.
    # note: handle motor sign flip.
    thetas = my_odd.get_motor_angles()*np.array([-1,1])
    thetas = thetas + CALIB_POSITION - CALIB_MEASUREMENT
    theta1, theta2 = thetas

    theta = 0.5*(theta1 + theta2)
    # x_foot and y_foot are unknown at this point and unused to compute r.
    q = np.array([0, theta1, theta2, 0, 0])
    r = model.leg_length(q, leg_params) # TODO: create q
    x_foot, y_foot = [r*np.sin(theta),-r*np.cos(theta)]
    q[[3,4]] = [x_foot, y_foot] # update q now that we know x_foot, y_foot.

    # Now compute qdot
    # x_foot_dot and y_foot_dot are unknown at this point and unused to compute r.
    # note: handle motor sign flip.
    theta1_dot, theta2_dot = my_odd.get_motor_velocities()*np.array([-1, 1])

    qdot = np.array([0, theta1_dot, theta2_dot, 0, 0])
    # Compute the jacobian of our leg's constraints.
    Da = controller.constraints_jac(q, leg_params)
    #print(f"Da shape is: {Da.shape}")
    #print(f"Da[0:2, 0:3] shape is: {Da[0:2, 0:3].shape}")

    qdot[[3, 4]] = np.linalg.solve(Da[0:2, 3:5], np.dot(-Da[0:2, 0:3], qdot[0:3]))

    print(f"theta1: {theta1:.3f} | theta2: {theta2:.3f}")

    return q, qdot



## Constants
LOOP_TIME_S = 0.5 # [sec]. How long between sending new data to the ODrive
#
#
#
## Julia's JIT compiler needs to compile every function the first time it sees it
## with a new function signature. Call everything here that we will need to
## call in the main loop.
#print("Compiling Julia Routines... ")
#output_torque = controller.stance_control(q, qdot, leg_params)


curr_time_s = time.perf_counter()
prev_time_s = curr_time_s
while True:
    curr_time_s = time.perf_counter()
    if (curr_time_s - prev_time_s) > LOOP_TIME_S:
        q, qdot = get_state()
        print(f"q[3,4] =    {q[3]:.3f}, {q[4]:.3f} | ", end="")
        print(f"qdot[3,4] = {qdot[3]:.3f}, {qdot[4]:.3f}")
#        # Get Measurements: flip sign on motor 0 to match world config.
#        raw_angles = np.array(my_odd.get_motor_angles()) * np.array([-1, 1]) # flip sign
#        # Create position state vector chunk.
#        q = thetas + CALIB_POSITION - CALIB_MEASUREMENT
#        qdot = np.array(my_odd.get_motor_velocities()) * np.array([-1, 1]) # flip sign.
#        #print(f"q is:    ({q[0]:.3f}, {q[1]:.3f})", end=" ")
#        #print(f"qdot is: ({qdot[0]:.3f}, {qdot[0]:.3f})")
#
#        # Update Motor Torques:
#        output_torque = controller.stance_control(q, qdot, leg_params)
#        output_current = output_torque/model.Ke
#        tf_current_a = current_a * np.array([-1, 1])
#        my_odd.set_torques(*tf_current_a) # should be in Amps.
#
#        # Setup next loop interation.
        prev_time_s = curr_time_s


