#!/usr/bin/env python3
"""Driver for controlling leg position"""

import cmd

from pareto_leg.pareto_leg import ParetoLeg


class UI(cmd.Cmd):

    def __init__(self, odrive, l1_len, l2_len):
        """constructor. Assumes odrive motors have already been pre-configured."""
        self.pareto_leg(odrive, l1_len, l2_len)

