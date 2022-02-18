#!/usr/bin/env python3
"""Driver for controlling leg position"""

from pareto_leg.five_bar_kins import FiveBarKinematics2D
from pareto_leg.odrive_driver import odrive_driver


class ParetoLeg(object):

    def __init__(self, odrive, l1_len, l2_len):
        """constructor. Assumes odrive motors have already been pre-configured."""
        self.odd = OdriveDriver(odrive)
        self.kins = FiveBarKinematics2D(l1_len, l2_len)


    def set_position(self, x, y):
        """Set the position of the robot leg in cartesian coordinates."""
        theta_axis0, theta_axis1 = self.kins.cartesian_to_joint(x, y)
        self.odd.set_positions(theta_axis0, theta_axis1)


    def get_position(self):
        """Get the position of the robot leg in cartesian coordinates."""
        theta_axis0, theta_axis1 = self.odd.get_positions()
        return self.kins.joint_to_cartesian(theta_axis0, theta_axis1)


    def get_joint_position(self):
        """Get the angular positionof each joint."""
        pass

    def get_joint_velocities(self):
        """Get the angular velocity of each joint."""
        pass
