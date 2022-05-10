import leg_controllers.model as model
import leg_controllers.hopper as hopper

class StanceController():
    def __init__(self, params, push_force, switching_time):
        self.params = params
        self.f = push_force 
        self.tau = switching_time
        self.t0 = 0.

    def initialize(self, q, qdot, t):
        self.t0 = t

    def control(self,q,qdot,t,dt):
        f = -self.f
        if t-self.t0 >= self.tau:
            f = self.f
        y = hopper.stance_anchor_projection(q,self.params)
        a = hopper.stance_template_dynamics(y,qdot[0])
        return hopper.stance_computed_torque(q,qdot,a+f,self.params)/model.Ke