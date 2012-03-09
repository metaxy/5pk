import numpy as np
import pylab

time_step = 0.1
time_overalle = 5
Q = 1e-5   # Process noise covariance
R = 0.1**2 # estimate of measurement noise covariance

xhat = [0,0]
xhatminus = [0,0]
P = np.matrix([1,0], [0,1])
Pminus = np.matrix([0,0], [0,0])
K = np.matrix([0,0], [0,0])

def measurement(time):
    return np.numpy.random.normal(np.sin(time),0.1)


