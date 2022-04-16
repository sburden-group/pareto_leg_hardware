import pickle
from matplotlib import pyplot as plt
import numpy as np

my_pickle = open("foo.pickle","rb")
data = np.array(pickle.load(my_pickle))
t = data[:,-1]
dt = data[:,-2]
y = data[:,4]
plt.plot(t,dt)
plt.show()
plt.plot(t,y)
plt.show()