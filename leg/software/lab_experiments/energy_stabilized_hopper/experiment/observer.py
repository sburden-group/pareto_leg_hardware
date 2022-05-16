import leg_controllers.model as model
import leg_controllers.hopper as hopper
from leg_controllers.designs import Params
import numpy as np

class Observer():
    """
    An implementation of a Kalman filter for the template dynamics.
    The filter is provided the template state measurement, and uses
    the template dynamics to produce an estimate of the template state.
    """
    def __init__(self,params: Params,Q,R,damping_fit):
        self.params = params
        self.Q = Q
        self.R = R
        self.x_pri = np.zeros(2)
        self.x_post = np.zeros(2)
        self.P_pri = np.zeros((2,2))
        self.P_post = np.zeros((2,2))
        self.zeta = damping_fit/(2*model.m_body*hopper.omega)

    def initialize(self, x, P):
        self.x_post = x
        self.x_pri = x
        self.P_post = P
        self.P_pri = P

    def prior_update(self, u, dt):
        # needs to be discrete time
        _A = np.array([
            [0., 1.],
            [-hopper.omega**2, -2*self.zeta*hopper.omega]
        ])
        A = np.eye(2)+dt*_A
        self.x_pri = A@self.x_post + dt*np.array([0,1])*(u-model.g)
        self.P_pri = A@self.P_post@A.T+self.Q

    def posterior_update(self, y):
        # innovation
        xres = y - self.x_pri
        Pres = self.P_pri + self.R
        K = self.P_pri@np.linalg.inv(Pres)
        self.x_post = self.x_pri+K@xres
        self.P_post = (np.eye(2)-K)@self.P_pri