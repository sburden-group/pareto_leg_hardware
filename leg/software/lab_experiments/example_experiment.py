#!/usr/bin/env python3
"""Example experiment."""
import odrive
from pareto_leg import ParetoLeg


if __name__ == "__main__":
    odrv0 = odrive.find_any()
    leg = ParetoLeg(odrv0, 90, 180)
    leg.cmdloop()
