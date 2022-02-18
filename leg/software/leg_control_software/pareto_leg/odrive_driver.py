#!/usr/bin/env python3
"""Driver for controlling both ODrive axes."""
from odrive.utils import dump_errors
from odrive.enums import *


class OdriveDriver(object):

    def __init__(self, odrive):
        """constructor. Assumes odrives have already been configured."""
        self.od = odrive
        self.control_mode = None
        self.reset()


    def reset(self):
        """initialize the motors."""
        self.od.clear_errors()
        self.od.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self.od.axis0.requested_state = en.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.od.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self.od.axis1.requested_state = en.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.set_position_control_mode()


# Modes:
    def set_mode(self, mode):
        self.od.axis0.controller.config.control_mode = mode
        self.od.axis1.controller.config.control_mode = mode


    def set_position_control_mode(self):
        self.set_mode(CONTROL_MODE_POSITION_CONTROL)
        self.mode = CONTROL_MODE_POSITION_CONTROL


    def set_torque_control_mode(self):
        self.set_mode(CONTROL_MODE_TORQUE_CONTROL)
        self.mode = CONTROL_MODE_TORQUE_CONTROL


# Homing:
    def zero_position(self):
        """Set the current position as (0,0)"""
        try:
            self.od.axis0.encoder.set_linear_position(0)
            self.od.axis1.encoder.set_linear_position(0)
        except Exception as e:
            print(e.message)
            dump_errors(self.od)


# Position and Torque Control Read and Writes:
    def set_torques(self, axis0_torque, axis1_torque):
        """set the torque on both motors in [Amps].
           Print message if we needed to switch modes first. Catch
           motor-related exceptions."""

        if self.mode != CONTROL_MODE_TORQUE_CONTROL:
            print("Switching to Torque Control Mode.")
            self.set_torque_control_mode()

        try:
            self.od.axis0.controller.input_torque = axis0_torque
            self.od.axis1.controller.input_torque = axis1_torque
        except Exception as e:
            print(e.message)
            dump_errors(e)


    def get_torques(self):
        """Read both torques."""
        return (self.od.axis0.motor.current_control.Iq_measured,
                self.od.axis1.motor.current_control.Iq_measured)


    def set_motor_angles(self, axis0_pos, axis1_pos):
        """Set the angles of both motors in radians.
           Print message if we needed to switch modes first. Catch
           motor-related exceptions."""

        if self.mode != CONTROL_MODE_POSITION_CONTROL:
            print("Switching to Position Control Mode.")
            self.set_position_control_mode()

        try:
            self.od.axis0.controller.input_pos = axis0_pos
            self.od.axis1.controller.input_pos = axis1_pos
        except Exception as e:
            print(e.message)
            dump_errors(self.od)


    def get_motor_angles(self):
        """Read both positions from each odrive axis.

        Note: looking top-down at the motor, CCW is positive rotation on both motors.

        """
        # *.pos_estimate is the estimated angle after running through the
        # state observer to correct encoder phase lag.
        return (self.od.axis0.encoder.pos_estimate,
                self.od.axis1.encoder.pos_estimate)

    def dump_errors(self):
        """Dump axis0 and axis1 errors."""
        dump_errors(self.od)
