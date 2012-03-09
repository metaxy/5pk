import pylab
import math
import random
import numpy

class KalmanFilter:
    def __init__(self, _A, _B, _H, _x, _P, _Q, _R):
        self.A = _A
        self.B = _B
        self.H = _H
        self.state_estimate = _x
        self.prob_estimate = _P
        self.Q = _Q
        self.R = _R
    def currentState(self):
        return self.state_estimate;
    def step(self, control_vector, measurement_vector):
        