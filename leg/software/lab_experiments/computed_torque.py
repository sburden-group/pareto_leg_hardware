#!/usr/bin/env python-jl
"""Demo for connecting to Julia."""

from pareto_leg.odrive_driver import OdriveDriver
import odrive

import time, sys
import numpy as np
from numpy import pi

# Connect to ODrive.
# Get Starting Position

# Hardware Connection Setup:
print("Connecting to ODrive... ")
odrv0 = odrive.find_any(timeout=1)
print("Connected!")
my_odd = OdriveDriver(odrv0)
my_odd.set_torque_control_mode()
my_odd.arm()

import julia
from julia import Pkg

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

def pd_controller(q,qdot,p):
    error = p-q[[3,4]]
    print(f'error: ({error[0]:.3f},{error[1]:.3f})')
    if q[1]-q[2]>0.6:
        Kp = 25
        Kd = 2.5
        command = Kp*(p-q[3:5])-Kd*qdot[3:5]
        u = controller.control(q,qdot,leg_params,command)
        return u
    else: # close to a hard stop, so we turn of Kd gain to avoid high frequency stimulus
        Kp = 25
        Kd = 0
        command = Kp*(p-q[3:5])-Kd*qdot[3:5]
        u = controller.control(q,qdot,leg_params,command)
        return u


## Constants
LOOP_TIME_S = 0.01 # [sec]. How long between sending new data to the ODrive
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
my_odd.arm()
waypoints = [[0.,-.20],[-0.15,-.20],[-0,-.27],[+.15,-.2]]
idx = 0
waypoint = waypoints[idx]
waypoint_update_time_s = curr_time_s
while True:
    curr_time_s = time.perf_counter()
    if curr_time_s - waypoint_update_time_s > 3. and idx < len(waypoints)-1:
        idx = idx + 1
        waypoint = waypoints[idx]
        waypoint_update_time_s = curr_time_s
    if (curr_time_s - prev_time_s) > LOOP_TIME_S:
        q, qdot = get_state()
#        u = pd_controller(q,qdot,waypoint)
        u = pd_controller(q,qdot,waypoints[idx])
        current_a = u / model.Ke
        my_odd.set_torques(*(current_a*np.array([-1,1])))
        prev_time_s = curr_time_s

#!/usr/bin/env python-jl
