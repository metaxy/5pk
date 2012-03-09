import numpy
import pylab

n_iter = 50

sz = (n_iter,)
x = -0.37727

z = numpy.random.normal(x,0.1,size=sz)

Q = 1e-5

xhat=numpy.zeros(sz)      # a posteri estimate of x
P=numpy.zeros(sz)         # a posteri error estimate
xhatminus=numpy.zeros(sz) # a priori estimate of x
Pminus=numpy.zeros(sz)    # a priori error estimate
K=numpy.zeros(sz)         # gain or blending factor

R = 0.1**2


xhat[0] = 0.0
P[0] = 1.0

for k in range(1,n_iter):
	xhatminus[k] = xhat[k-1]
	Pminus[k] = P[k-1] + Q
		
	K[k] = Pminus[k] / (Pminus[k] + R )
	xhat[k] = xhatminus[k] + K[k] * (z[k] - xhatminus[k])
	P[k] = (1-K[k])*Pminus[k]
	
	print "xhatminus" , xhatminus[k]
	print "PMinus" , Pminus[k]
	print "k", K[k]
	print "xhat", xhat[k]
	print "P",  P[k]

	print 

pylab.figure()
pylab.plot(z,'k+',label='noisy measurements')
pylab.plot(xhat,'b-',label='a posteri estimate')
pylab.plot(xhatminus,'c-', label="xhat minus")
pylab.plot(Pminus,'r-', label="p-")
pylab.plot(K,'y-', label="K")
pylab.plot(P,'g-', label="P")

pylab.axhline(x,color='g',label='truth value')
pylab.legend()
pylab.xlabel('Iteration')
pylab.ylabel('Voltage')
#pylab.figure()

#valid_iter = range(1,n_iter) # Pminus not valid at step 0
#pylab.plot(valid_iter,Pminus[valid_iter],label='a priori error estimate')
#pylab.xlabel('Iteration')
#pylab.ylabel('$(Voltage)^2$')
#pylab.setp(pylab.gca(),'ylim',[0,.01])
pylab.show()
