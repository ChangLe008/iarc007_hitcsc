# -*- coding=utf-8 -*-
# Kalman filter example demo in Python
#coding:utf-8
import numpy
import pylab
import math
import string
#这里是假设A=1，H=1的情况
a = open('/home/hitcsc/catkin_ws/log/iarc/kf.txt')
text = a.readlines();
dh=[];
dt=[];
dx=[];
dy=[]
dxf=[]
dtt=[]
for b in text:
    c = b.split("\t");
    #print c;
    dx.append(-1*string.atof(c[0]));
    dy.append(string.atof(c[1])-3);


ddx = dx[:200]#[50:155]
# intial parameters
n_iter = len(ddx)
sz = (n_iter,) # size of array
x = -0.11 # truth value (typo in example at top of p. 13 calls this z)
z = ddx # observations (normal about x, sigma=0.1)
# z = numpy.random.normal(x,0.05,size=sz)
Q = 1e-5 # process variance

# allocate space for arrays
xhat=numpy.zeros(sz)      # a posteri estimate of x
P=numpy.zeros(sz)         # a posteri error estimate
xhatminus=numpy.zeros(sz) # a priori estimate of x
Pminus=numpy.zeros(sz)    # a priori error estimate
K=numpy.zeros(sz)         # gain or blending factor

R = 0.1**2 # estimate of measurement variance, change to see effect

# intial guesses
xhat[0] = 0.0
P[0] = 1.0
x=sum(dx[:150])/len(dx[:150])
for k in range(1,n_iter):
    # time update
    xhatminus[k] = xhat[k-1]  #X(k|k-1) = AX(k-1|k-1) + BU(k) + W(k),A=1,BU(k) = 0
    Pminus[k] = P[k-1]+Q      #P(k|k-1) = AP(k-1|k-1)A' + Q(k) ,A=1

    # measurement update
    K[k] = Pminus[k]/( Pminus[k]+R ) #Kg(k)=P(k|k-1)H'/[HP(k|k-1)H' + R],H=1
    xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k]) #X(k|k) = X(k|k-1) + Kg(k)[Z(k) - HX(k|k-1)], H=1
    P[k] = (1-K[k])*Pminus[k] #P(k|k) = (1 - Kg(k)H)P(k|k-1), H=1

pylab.figure()
pylab.plot(z,'k+-.',label='Noisy measurements',linewidth=2)     #测量值
pylab.plot(xhat,'b-',label='Eestimation',linewidth=2)  #过滤后的值
pylab.axhline(x,linestyle="--",color='g',label='Ground truth',linewidth=2)    #系统值
pylab.legend()
pylab.xlabel('Iteration')
pylab.ylabel('Distance/s')
pylab.title('Kalman filter')
pylab.figure()
valid_iter = range(1,n_iter) # Pminus not valid at step 0
pylab.plot(valid_iter,Pminus[valid_iter],label='a priori error estimate')
pylab.xlabel('Iteration')
pylab.ylabel('Estimate error/m')
pylab.setp(pylab.gca(),'ylim',[0,.01])
pylab.title('Estimate error')
pylab.show()