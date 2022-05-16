class PIDController():
    def __init__(self,Kp,Ki,Kd,tau):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tau = tau
        self.integrator = 0.
    
    def update(self, r, x, xdot, dt):
        error = r - x
        self.integrator = self.integrator*(1-dt/self.tau)+dt*error
        return self.Kp*error + self.Ki*self.integrator - self.Kd*xdot

    def reset(self):
        self.integrator = 0.