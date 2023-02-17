from .pid_controller import PIDController
import leg_controllers.model as model
import leg_controllers.hopper as hopper
import numpy as np

class FlightController():
    def __init__(self, leg_params):
        self.params = leg_params
        self.pid = PIDController(225,200.,5.,10.)
        self.setpoint = np.array([0.,0.])
    
    def initialize(self):
        self.pid.reset()
    
    def output(self, q, qdot, dt):
        p = hopper.flight_anchor_projection(q,self.params)
        pdot = hopper.flight_anchor_pushforward(q,self.params) @ qdot
        u = self.pid.update(self.setpoint,p,pdot,dt)
        return hopper.flight_computed_torque(q,qdot,u,self.params)/model.Ke