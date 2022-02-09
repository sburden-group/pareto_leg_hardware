#!/usr/bin/env python3
"""Pareto Leg Class"""

# include some odrive stuff as needed.

class ParetoLeg(Object):

    def __init__(self, l1, l2, odrive):
        self.l1 = l1
        self.l2 = l2

    def forward_kinematics(self, joint_vector):
        """Given joint angle vector, compute the XY position.

        Note: the coordinate frame origin for the robot is located at the center
              between the two motors.

        Note: looking at the motor top-down, ccw rotation is positive.

        Note: leg links are not allowed to cross, so there is only one solution
        """
        # make some aliases for readability
        l1 = self.l1
        l2 = self.l2
        theta2, theta4 = joint_vector
        # Geometric Relations:
        #   Internal angles sum to 2pi.
        #   femur and tibya lengths on "leglets" are the same.

        # Calculate virtual link length from origin to end effector.
        theta3 = math.pi - (theta2 + theta4)
        theta6 = 0.5*(theta2 - theta4)
        l3 = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(theta3)) # law o cosines

        # Polar to cartesian:
        x = l3*math.sin(theta6)
        y = -l3*math.cos(theta6)

        return (x,y)

    def inverse_kinematics(self, x_vector):
        """Given XY leg state, compute the joint angles.

        Note: There are technically 2 solutions, but only one is feasible bc
              of hardware constraints.
        """
        # Make some alieses for readability.
        x,y = x_vector
        l1 = self.l1
        l2 = self.l2

        l3 = math.sqrt(x**2 + y**2)
        # flipped args so our angle is measured from lower vertical segment
        theta6 = math.atan2(x,y)
        theta7 = math.acos((l1**2 + l3**2 - l2**2)/(2*l1*l3)) # law o cosines
        theta2 = theta7 + theta6
        theta4 = theta7 - theta6

        return (theta2, theta4)


    # Wrappers for foward/inverse kinematics:
    def joint_to_cartesian(self, joint_vector):
        return forward_kinematics(joint_vector)


    def cartesian_to_joint(self, x_vector):
        return inverse_kinematics(x_vector)


    # Control Hardware
