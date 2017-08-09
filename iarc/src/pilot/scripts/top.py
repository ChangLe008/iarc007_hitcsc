# -*- coding=utf-8 -*-
# Kalman filter example demo in Python
#coding:utf-8
import numpy
import pylab
import math
import string
import matplotlib.pyplot as plt
x = [17,16,15,16,17,9,8,13,10,17]
y = [16,17,19,20,21,22,23,23,24,23]
fig = plt.figure(1)
plt.plot(x,y)
plt.show()