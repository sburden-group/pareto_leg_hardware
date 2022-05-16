import numpy as np
from scipy.linalg import expm 
import leg_controllers.hopper as hopper
import leg_controllers.model as model

A = np.array([
    [0., 1., 0.],
    [-hopper.omega**2, 0., -model.g],
    [0.,0.,0.]
])

def reference(E,y0,t):
    # calculate initial velocity from E,y0
    v0 = -np.sqrt(2*(E-.5*(hopper.omega**2)*(y0**2)-model.g*y0))
    x0 = np.array([y0,v0,1.])
    return np.array([(expm(A*t)@x0)[0] for t in t])
