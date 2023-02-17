from .pid_controller import PIDController
import leg_controllers.model as model
import leg_controllers.hopper as hopper
import numpy as np

class StanceController():
    def __init__(self, leg_params):
        self.params = leg_params
        self.pid = PIDController(225.,100.,7.5,30.)
    
    def initialize(self):
        self.pid.reset()
    
    def output(self, q, qdot, dt):
        setpoint = np.array([0.25-self.params.l1-self.params.l2-model.foot_offset])
        p = hopper.stance_anchor_projection(q,self.params)
        pdot = hopper.stance_anchor_pushforward() @ qdot
        u = self.pid.update(setpoint,p,pdot,dt)
        return hopper.stance_computed_torque(q,qdot,u,self.params)/model.Ke