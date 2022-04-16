from pareto_leg.odrive_driver import OdriveDriver
import odrive

import time
import numpy as np
from numpy import pi

import julia

CONTROL_LOOP_TIME_S = 0.01 # seconds (update rate of 10 Hz)

class PIDController(object):

    def __init__(self,Kp,Ki,Kd,tau):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tau = tau
        self.integrator = 0.

    def update(self, p, q, qdot, dt):
        error = p - q
        self.integrator = self.integrator*(1-dt*self.tau)+dt*error
        return self.Kp*error + self.Ki*self.integrator - self.Kd*qdot

    def reset(self):
        self.integrator = 0.
