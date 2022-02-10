#!/usr/bin/env/python3
import pytest
import numpy as np
from pareto_leg.five_bar_kins import FiveBarKinematics2D

l1 = 90 # [mm]
l2 = 180 # [mm]
kins = FiveBarKinematics2D(l1, l2)

def test_fw_kin_standing_pose():
    joints = (0, 0) # The leg is pointing straight down.
    cartesian_answer = (0, -270)
    assert np.allclose(kins.joint_to_cartesian(joints), cartesian_answer)


def test_fw_kin_left_and_right_extremes():
    joints = (np.pi/2, -np.pi/2) # The leg is aligned with the +x direction
    cartesian_answer = (270, 0)
    assert np.allclose(kins.joint_to_cartesian(joints), cartesian_answer)

    joints = (-np.pi/2, np.pi/2) # The leg is aligned with the -x direction
    cartesian_answer = (-270, 0)
    assert np.allclose(kins.joint_to_cartesian(joints), cartesian_answer)


def test_fw_kin_spread_femurs():
    joints = (np.pi/2, np.pi/2) # femurs are horizontal in opposite directions
    cartesian_answer = (0, -np.sqrt(l2**2 - l1**2))
    assert np.allclose(kins.joint_to_cartesian(joints), cartesian_answer)


def test_inv_kin_standing_pose():
    x_vector = (0, -270)
    joint_answer = (0,0)
    assert np.allclose(kins.cartesian_to_joint(x_vector), joint_answer)

def test_inv_kin_left_and_right_extremes():
    x_vector = (270, 0)
    joint_answer = (np.pi/2, -np.pi/2) # The leg is aligned with the +x direction
    assert np.allclose(kins.cartesian_to_joint(x_vector), joint_answer)

    x_vector = (-270, 0)
    joint_answer = (-np.pi/2, np.pi/2) # The leg is aligned with the -x direction
    assert np.allclose(kins.cartesian_to_joint(x_vector), joint_answer)

def test_inv_kin_spread_femurs():
    joint_answer = (np.pi/2, np.pi/2)
    x_vector = (0, -np.sqrt(l2**2 - l1**2))
    assert np.allclose(kins.cartesian_to_joint(x_vector), joint_answer)

