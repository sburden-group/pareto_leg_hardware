#!/usr/bin/env python3
"""Driver for controlling leg position"""

from pareto_leg.five_bar_kins import FiveBarKinematics2D
from pareto_leg.odrive_driver import odrive_driver


class ParetoLeg(object):

    def __init__(self, odrive, l1_len, l2_len):
        """constructor. Assumes odrive motors have already been pre-configured."""
        self.odd = OdriveDriver(odrive)
        self.kins = FiveBarKinematics2D(l1_len, l2_len)


    def set_cartesian_position(self, x, y):
        """Set the position of the robot leg in cartesian coordinates."""
        # Kinematic model needs to be fixed (sign error on motor angles)
        # based on leg configuration
        -theta_axis0, theta_axis1 = self.kins.cartesian_to_joint(x, y)
        self.odd.set_positions(theta_axis0, theta_axis1)


    def get_cartesian_position(self):
        """Get the position of the robot leg in cartesian coordinates."""
        theta_axis0, theta_axis1 = self.odd.get_positions()
        return self.kins.joint_to_cartesian(theta_axis0, theta_axis1)


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


    def get_joint_velocities(self):
        """Get the angular velocity of each joint."""
        pass # TODO!
