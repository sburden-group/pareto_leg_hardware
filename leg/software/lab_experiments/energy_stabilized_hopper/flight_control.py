from pid_controller import PIDController
import leg_controllers.model as model
import leg_controllers.hopper as hopper
import numpy as np

class FlightController():
    def __init__(self, leg_params):
        self.params = leg_params
        self.pid = PIDController(7.5,0.,0.1,30.)
        self.last_p = np.array([np.NaN]) 
    
    def initialize(self):
        self.last_p = np.array([np.NaN]) 
        self.pid.reset()
    
    def output(self, q, qdot, dt):
        p = hopper.flight_anchor_projection(q,self.params)
        pdot = hopper.flight_anchor_pushforward(q,self.params) @ qdot
        if np.any(self.last_p==np.NaN):
            self.last_p = p
        u = self.pid.update(np.zeros(2),p,pdot,dt)
        # spring_current = hopper.flight_control(q,qdot,self.params)/model.Ke
        spring_current = np.zeros(2)
        return spring_current + np.array([u[0]+u[1],-u[0]+u[1]])
        # return hopper.flight_computed_torque(q,qdot,a,self.params)/model.Ke