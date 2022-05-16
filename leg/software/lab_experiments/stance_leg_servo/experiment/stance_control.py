from .pid_controller import PIDController
import leg_controllers.model as model
import leg_controllers.hopper as hopper
import numpy as np

class StanceController():
    def __init__(self, leg_params):
        self.params = leg_params
        self.pid = PIDController(125.,75.,3.0,30.)
    
    def initialize(self):
        self.pid.reset()
    
    def output(self, q, qdot, dt):
        setpoint = np.array([0.22-self.params.l1-self.params.l2-model.foot_offset])
        p = hopper.stance_anchor_projection(q,self.params)
        pdot = hopper.stance_anchor_pushforward() @ qdot
        u = self.pid.update(setpoint,p,pdot,dt)
        # spring_current = hopper.flight_control(q,qdot,self.params)/model.Ke
        # return spring_current + np.array([u[0]+u[1],-u[0]+u[1]])
        # spring_current = np.zeros(2)
        # return spring_current+np.array([u[0]+u[1],-u[0]+u[1]])
        return hopper.stance_computed_torque(q,qdot,u,self.params)/model.Ke