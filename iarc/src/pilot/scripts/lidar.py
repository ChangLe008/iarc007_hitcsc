#coding:UTF-8

"""
Created on 2017/04/07
@author: Leonidas
"""
import matplotlib.pyplot as plt
import string
import math
#ResultFormat = "%0.4f"
#Usage: ResultFormat%(1/3)
str = '/home/hitcsc/catkin_ws/log/iarc/lidar.txt'
a = open(str)
text = a.readlines()
obs_num = 0
class obs:
    def __init__(self):
        self.real = 0       # 可靠性
        self.x = 0
        self.y = 0
        self.serial = 0
        self.lost_counter = 0

t_m = []
x = []
y = []
temp_x = []
temp_y = []
x0 = []
y0 = []
x1 = []
y1 = []
for ctdata in text:
    ct = ctdata.split("\t")
    t_m.append(string.atoi(ct[0]))
    for item in range(int(string.atof(ct[0]))):
        temp_x.append(string.atof(ct[1+item*2]))
        temp_y.append(string.atof(ct[2+item*2]))
    x.append(temp_x)
    y.append(temp_y)
    temp_x = []
    temp_y = []

obstacle = []
for lenth in range(len(t_m)):
    abc_instance = [obs() for i in range(t_m[lenth])]
    counter = 1
    for i in range(t_m[lenth]):
        if(math.sqrt(x[lenth][i]*x[lenth][i]+y[lenth][i]*y[lenth][i])<=0.5):
            abc_instance[i].real=0
        else:
            abc_instance[i].real=1
            abc_instance[i].x=x[lenth][i]
            abc_instance[i].y=y[lenth][i]
            abc_instance[i].serial=counter
            counter = counter + 1
    if len(obstacle)==0:
        for i in range(t_m[lenth]):
            obs_temp = 0
            if abc_instance[i].real==1:
                obstacle.append(abc_instance[i])
                obstacle[obs_temp].serial= obs_temp+1
                obs_temp = obs_temp+1
    # elif counter == len(obstacle):
    #     12
    else:
        temp_dis = []
        for h in range(len(obstacle)):
            for i in range(t_m[lenth]):
                if abc_instance[i].real==1:
                    temp_dis.append(math.sqrt((abc_instance[i].x-obstacle[h].x)*(abc_instance[i].x-obstacle[h].x)+(abc_instance[i].y-obstacle[h].y)*(abc_instance[i].y-obstacle[h].y)))
                else:
                    temp_dis.append(100000)
            aa = temp_dis.index(min(temp_dis))
            if aa <=0.05:
                obstacle[h].x = abc_instance[aa].x
                obstacle[h].y = abc_instance[aa].y
                obstacle[h].lost_counter = 0
                obstacle[h].real = 1
                print obstacle[h].x,obstacle[h].y,h
            elif obstacle[h].lost_counter>=10:
                obstacle[h].real = 0
            else:
                obstacle[h].lost_counter = obstacle[h].lost_counter+1
            temp_dis = []
        # x0.append(obstacle[0].x)
        # y0.append(obstacle[0].y)
        x1.append(obstacle[2].x)
        y1.append(obstacle[2].y)
# plt.plot(ttt,ul,'k',label='ul')
# plt.legend()
# plt.grid()
# plt.show()
# print xs
plt.plot(x1,y1,'r')
plt.plot(x0,y0,'b2')
plt.grid()
plt.show()
print abc_instance[0].lost_counter