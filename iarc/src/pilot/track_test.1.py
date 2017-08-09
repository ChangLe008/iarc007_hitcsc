#coding:UTF-8

"""
Created on 2017/04/07
@author: Leonidas
"""
import matplotlib.pyplot as plt
import string
import math

def as_num(x):
    y='{:.5f}'.format(x) # 5f表示保留5位小数点的float型
    return(y)

#ResultFormat = "%0.4f"
#Usage: ResultFormat%(1/3)
a = open('/home/hitcsc/catkin_ws/log/test_client/sona.txt')
text = a.readlines()
h=[]
t=[]
x=[]
y=[]
vx=[]
vy=[]
target = []
flag=1
ro=[]
po=[]
yo=[]

tt = []
cp=[]
cr=[]
rr=[]
pr=[]
yr=[]
cvx=[]
cvy=[]
time = 0
for b in text:
    c = b.split("\t")
    #print c;
    time = time +1
    t.append(time)
    target.append(string.atof(c[0]))
    x.append(string.atof(c[1]))

temp= target[0]
temp1 = x[0]
for i in range(len(target)):
    # target[i] = target[i]-temp
    x[i] = x[i]-temp1
fig = plt.figure(1)
plt.plot(t,target,'g')
plt.plot(t,x,'r')
plt.grid()
plt.show()
