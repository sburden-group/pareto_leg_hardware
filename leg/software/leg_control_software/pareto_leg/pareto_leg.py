#!/usr/bin/env python3
"""Driver for controlling leg position"""
from inpromptu import Inpromptu, cli_method

from .five_bar_kins import FiveBarKinematics2D
from .odrive_driver import OdriveDriver
import odrive


class ParetoLeg(Inpromptu):
#class ParetoLeg(object):

    # constants
    CALIB_ANGLE_DEGS = [90, 90]


    def __init__(self, odrive, l1_len, l2_len):
        """constructor. Assumes odrive motors have already been pre-configured."""
        super().__init__()
        self.odd = OdriveDriver(odrive)
        self.kins = FiveBarKinematics2D(l1_len, l2_len)


    def set_cartesian_position(self, x, y):
        """Set the position of the robot leg in cartesian coordinates."""
        theta_axis0, theta_axis1 = self.kins.cartesian_to_joint(x, y)
        # Kinematic model assumes flipped angle0 from how it is installed
        # based on leg configuration.
        theta_axis0 = -theta_axis0
        self.odd.set_positions(theta_axis0, theta_axis1)


    @cli_method
    def get_cartesian_position(self):
        """Get the position of the robot leg in cartesian coordinates."""
        theta_axis0, theta_axis1 = self.odd.get_motor_angles() # radians.
        return self.kins.joint_to_cartesian([theta_axis0, theta_axis1])


    @cli_method
    def get_joint_angles(self):
        """Get the angular position of each joint according to configuration assumptions.

        Note: with robot orientation is Motor0 closer to Motor1, both motor
              angles are positive when the leg moves CCW in this orientation.

        """
        motor_angles = self.odd.get_motor_angles()
        # Handle Joey configuration convention.
        return (-motor_angles[0], motor_angles[1])


    def set_joint_angles(self, motor0_theta, motor1_theta):
        """Set the angular position of each joint according to configuration assumptions.

        Angle units are in radians.
        """
        self.odd.set_motor_angles(-motor0_theta, motor1_theta)


    @cli_method
    def apply_calibration(self):
        """Tell the ODrive that the current position is the calibration position."""

        # Configure the motor in joint space.
        # i.e: CCW rotation is positive angle looking top-down at each motor.
        self.odd.configure_motor_angle_degs_as(ParetoLeg.CALIB_ANGLE_DEGS[0],
                                               ParetoLeg.CALIB_ANGLE_DEGS[1])


    def get_joint_velocities(self):
        """Get the angular velocity of each joint."""
        pass # TODO!
