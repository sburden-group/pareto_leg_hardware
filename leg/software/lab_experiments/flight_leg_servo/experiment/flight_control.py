from .pid_controller import PIDController
import leg_controllers.model as model
import leg_controllers.hopper as hopper
import numpy as np

class FlightController():
    def __init__(self, leg_params):
        self.params = leg_params
        self.pid = PIDController(30.,0.,2.5,30.)
        self.last_p = np.array([np.NaN]) 
    
    def initialize(self):
        self.last_p = np.array([np.NaN]) 
        self.pid.reset()
    
    def output(self, q, qdot, dt):
        setpoint = np.array([0.,0.])
        p = hopper.flight_anchor_projection(q,self.params)
        pdot = hopper.flight_anchor_pushforward(q,self.params) @ qdot
        a = self.pid.update(setpoint,p,pdot,dt)
        return hopper.flight_computed_torque(q,qdot,a,self.params)/model.Ke