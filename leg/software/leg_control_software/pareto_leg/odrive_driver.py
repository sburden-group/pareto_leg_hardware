#!/usr/bin/env python3
"""Driver for controlling both ODrive axes."""
from inpromptu import Inpromptu, cli_method

from math import pi

from odrive.utils import dump_errors
from odrive.enums import *
import odrive


#class OdriveDriver(object):
class OdriveDriver(Inpromptu):

    def __init__(self, odrive):
        """constructor.

        Assumes odrives have already been configured according to the bringup
        script."""
        super().__init__()
        self.od = odrive
        self.control_mode = None
        self.reset()


    @cli_method
    def reset(self):
        """initialize the motors."""
        self.od.clear_errors()
        self.set_position_control_mode()
        self.od.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self.od.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self.disarm()


    @cli_method
    def arm(self):
        """Arm the leg motors."""
        self.od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.od.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


    @cli_method
    def disarm(self):
        """disarm the leg motors."""
        self.od.axis0.requested_state = AXIS_STATE_IDLE
        self.od.axis1.requested_state = AXIS_STATE_IDLE


# Modes:
    def set_mode(self, mode):
        self.od.axis0.controller.config.control_mode = mode
        self.od.axis1.controller.config.control_mode = mode


    @cli_method
    def set_position_control_mode(self):
        """Wrapper for setting position control mode."""
        self.set_mode(CONTROL_MODE_POSITION_CONTROL)
        self.control_mode = CONTROL_MODE_POSITION_CONTROL


    @cli_method
    def set_torque_control_mode(self):
        """Wrapper for setting torque control mode."""
        self.set_mode(CONTROL_MODE_TORQUE_CONTROL)
        self.control_mode = CONTROL_MODE_TORQUE_CONTROL


# Homing:
    @cli_method
    def zero_position(self):
        """Set the current position as (0,0)"""
        try:
            self.od.axis0.encoder.set_linear_count(0)
            self.od.axis1.encoder.set_linear_count(0)
        except Exception as e:
            print(e.message)
            dump_errors(self.od)


# Position and Torque Control Read and Writes:
    @cli_method
    def set_torques(self, axis0_torque: float, axis1_torque: float):
        """set the torque on both motors in [Amps].
           Print message if we needed to switch modes first. Catch
           motor-related exceptions."""

        if self.control_mode != CONTROL_MODE_TORQUE_CONTROL:
            print("Switching to Torque Control Mode.")
            self.set_torque_control_mode()

        try:
            self.od.axis0.controller.input_torque = axis0_torque
            self.od.axis1.controller.input_torque = axis1_torque
        except Exception as e:
            print(e.message)
            dump_errors(e)


    @cli_method
    def get_torques(self):
        """Read both torques."""
        return (self.od.axis0.motor.current_control.Iq_measured,
                self.od.axis1.motor.current_control.Iq_measured)


    def set_motor_angles(self, axis0_angle_rad, axis1_angle_rad):
        """Set the angles of both motors in radians.
           Print message if we needed to switch modes first. Catch
           motor-related exceptions."""

        if self.control_mode != CONTROL_MODE_POSITION_CONTROL:
            print("Switching to Position Control Mode.")
            self.set_position_control_mode()

        try:
            # Convert radians to revolutions.
            self.od.axis0.controller.input_pos = axis0_angle_rad / (2 * pi)
            self.od.axis1.controller.input_pos = axis1_angle_rad / (2 * pi)
        except Exception as e:
            print(str(e))
            print("ODrive Status Dump:")
            dump_errors(self.od)


    @cli_method
    def set_motor_angles_deg(self, axis0_angle_deg: float, axis1_angle_deg: float):
        return self.set_motor_angles(axis0_angle_deg * (pi/180),
                                     axis1_angle_deg * (pi/180))


    @cli_method
    def get_motor_angles(self):
        """Read both angles (in radians) from each odrive axis.

        Note: looking top-down at the motor, CCW rotation is positive on each motor.
        Note: Angles (radians) are not clamped and not calibrated.

        """
        # *.pos_estimate is the estimated angle after running through the
        # state observer to correct encoder phase lag.
        return (self.od.axis0.encoder.pos_circular* 2 * pi,
                self.od.axis1.encoder.pos_circular* 2 * pi)


    @cli_method
    def get_motor_velocities(self):
        """Read both angle velocities (in radians) from each odrive axis.

        Note: looking top-down at the motor, CCW rotation is positive on each motor.

        """
        # *.pos_estimate is the estimated angle after running through the
        # state observer to correct encoder phase lag.
        return (self.od.axis0.encoder.vel_estimate * 2 * pi,
                self.od.axis1.encoder.vel_estimate * 2 * pi)


    def configure_motor_angles_as(self, axis0_angle_rad: float, axis1_angle_rad: float):
        """Set the current motor angles to be a specific value in radians."""

        axis0_cpr = self.od.axis0.encoder.config.cpr
        axis1_cpr = self.od.axis1.encoder.config.cpr

        curr_pos_rad = self.get_motor_angles()

        #new_angle_rad = [curr_pos_rad[0] + axis0_angle_rad,
        #                 curr_pos_rad[1] + axis1_angle_rad]
        new_angle_rad = [axis0_angle_rad,
                         axis1_angle_rad]

        # Clamp above values to (-pi, +pi)
        for index, angle in enumerate(new_angle_rad):
            while new_angle_rad[index] < -pi:
                new_angle_rad[index] += 2*pi
            while new_angle_rad[index] > pi:
                new_angle_rad[index] -= 2*pi

        self.od.axis0.encoder.set_linear_count(new_angle_rad[0]/(2*pi)*axis0_cpr)
        self.od.axis1.encoder.set_linear_count(new_angle_rad[1]/(2*pi)*axis1_cpr)


    @cli_method
    def configure_motor_angle_degs_as(self, axis0_angle_deg: float, axis1_angle_deg: float):
        self.configure_motor_angles_as(axis0_angle_deg * (pi/180),
                                       axis1_angle_deg * (pi/180))


    @cli_method
    def dump_errors(self):
        """Dump axis0 and axis1 errors."""
        dump_errors(self.od)


if __name__ == "__main__":

    odrv0 = odrive.find_any()
    my_odd = OdriveDriver(odrv0)

    my_odd.cmdloop()
