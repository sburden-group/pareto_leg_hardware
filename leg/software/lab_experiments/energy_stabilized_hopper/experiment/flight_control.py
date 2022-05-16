from .pid_controller import PIDController
import leg_controllers.model as model
import leg_controllers.hopper as hopper
import numpy as np

class FlightController():
    def __init__(self, leg_params):
        self.params = leg_params
        self.pid = PIDController(225,200.,0.,10.)
        self.setpoint = np.array([np.pi/8,np.pi/8-.2])
    
    def initialize(self):
        self.pid.reset()
    
    def output(self, q, qdot, dt):
        p = hopper.flight_anchor_projection(q,self.params)
        pdot = hopper.flight_anchor_pushforward(q,self.params) @ qdot
        u = self.pid.update(np.zeros(2),p,pdot,dt)
        # feed_forward_current = np.array([-.5,-.5])
        # spring_current = hopper.flight_control(q,qdot,self.params)/model.Ke
        return hopper.flight_computed_torque(q,qdot,u,self.params)/model.Ke