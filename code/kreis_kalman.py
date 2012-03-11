import pylab
import math
import random
import numpy

class KalmanFilter:
    def __init__(self, _F, _B, _H, _x, _P, _Q, _R):
        self.F = _F             # State transition matrix.
        self.B = _B             # Control matrix.
        self.H = _H             # Observation matrix.
        self.xhat = _x          # Initial state estimate.
        self.P = _P # Initial covariance estimate.
        self.Q = _Q             # Estimated error in process.
        self.R = _R             # Estimated error in measurements.
    def currentState(self):
        return self.xhat;
    def step(self, u, z):
        # u: control vector, z: measurement vector
        
        #predict
        xhatminus = self.F * self.xhat + self.B * u
        Pminus = (self.F * self.P) * numpy.transpose(self.F) + self.Q
        
        #observation
        
        # i dont know why but the second one y is much better in estimating
        # but the first one should be according to wikipedia the correct one
        #y = z - self.H * xhatminus # innovation
        y = self.H * z - xhatminus # innovation
        
        
        S = self.H * Pminus * numpy.transpose(self.H) + self.R # innovation covariance
        
        #update
        
        K = Pminus * numpy.transpose(self.H) * numpy.linalg.inv(S)
        self.xhat = xhatminus + K * y
        
        self.P = (numpy.eye(self.P.shape[0]) - K * self.H) * Pminus
        
        
class Cannon:
    #--------------------------------VARIABLES----------------------------------
    angle = 60
    muzzle_velocity = 100
    gravity = [0,-9.81]
    velocity = [muzzle_velocity*math.cos(angle*math.pi/180), muzzle_velocity*math.sin(angle*math.pi/180)]
    loc = [0,0]
    acceleration = [0,0]
    #---------------------------------METHODS-----------------------------------
    def __init__(self,_timeslice,_noiselevel):
        self.timeslice = _timeslice
        self.noiselevel = _noiselevel
    def add(self,x,y):
        return x + y
    def mult(self,x,y):
        return x * y
    def GetX(self):
        return self.loc[0]
    def GetY(self):
        return self.loc[1]
    def GetXWithNoise(self):
        return random.gauss(self.GetX(),self.noiselevel)
    def GetYWithNoise(self):
        return random.gauss(self.GetY(),self.noiselevel)
    def GetXVelocity(self):
        return self.velocity[0]
    def GetYVelocity(self):
        return self.velocity[1]
    # Increment through the next timeslice of the simulation.
    def Step(self):

        timeslicevec = [self.timeslice,self.timeslice]
        sliced_gravity = map(self.mult,self.gravity,timeslicevec)
        sliced_acceleration = sliced_gravity
        self.velocity = map(self.add, self.velocity, sliced_acceleration)
        sliced_velocity = map(self.mult, self.velocity, timeslicevec )
        self.loc = map(self.add, self.loc, sliced_velocity)
        if self.loc[1] < 0:
            self.loc[1] = 0
            
class System
    def __init__ (self, _timeslice):
        self.timeslice _timeslice
    
    def step(self):
    
    # return vector x
    def getX(self):
        return [0,0]
    #return vector z
    def measureX(self):
        

timeslice = 0.01
iterations = 1444 

real_x = []
real_y = []
measure_x = []
measure_y = []
k_x = []
k_y = []

speedX = muzzle_velocity*math.cos(angle*math.pi/180)
speedY = muzzle_velocity*math.sin(angle*math.pi/180)

F = numpy.matrix([[1,timeslice,0,0],[0,1,0,0],[0,0,1,timeslice],[0,0,0,1]])
B = numpy.matrix([[0,0,0,0],[0,0,0,0],[0,0,1,0],[0,0,0,1]])

H = numpy.eye(4)
xhat = numpy.matrix([[0],[speedX*2],[500],[speedY*2]])
P = numpy.eye(4)
Q = numpy.eye(4)*0.001
R = numpy.eye(4)*4
kf = KalmanFilter(F, B, H, xhat, P, Q, R)


control_vector = numpy.matrix([[0],[0],[-9.81*timeslice*timeslice],[-9.81*timeslice]])

for i in range(iterations):
    x.append(c.GetX())
    y.append(c.GetY())
    newestX = c.GetXWithNoise()
    newestY = c.GetYWithNoise()
    nx.append(newestX)
    ny.append(newestY)

    c.Step()
    kx.append(kf.currentState()[0,0])
    ky.append(kf.currentState()[2,0])
    kf.step(control_vector,numpy.matrix([[newestX],[c.GetXVelocity()],[newestY],[c.GetYVelocity()]]))

# Plot all the results we got.
pylab.plot(x,y,'-',nx,ny,':',kx,ky,'--')
pylab.xlabel('X position')
pylab.ylabel('Y position')
pylab.title('Measurement of a Cannonball in Flight')
pylab.legend(('true','measured','kalman'))
pylab.show()
