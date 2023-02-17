import leg_controllers.model as model
import leg_controllers.hopper as hopper
import numpy as np

class StanceController():
    def __init__(self, params, pull_force, push_force, switching_time):
        self.params = params
        self.pull_force = pull_force 
        self.push_force = push_force 
        self.tau = switching_time
        self.t0 = 0.

    def initialize(self, q, qdot, t):
        self.t0 = t

    def control(self,q,qdot,t,dt):
        if self.t0 < 0.:
            self.initialize(q,qdot,t)
        f = self.pull_force 
        if 2*self.tau > t-self.t0 >= self.tau:
            f = self.push_force
        elif t-self.t0 >= 2*self.tau:
           f = 0. 
        y = hopper.stance_anchor_projection(q,self.params)
        a = hopper.stance_template_dynamics(y,qdot[0])
        return hopper.stance_computed_torque(q,qdot,a+f,self.params)/model.Ke