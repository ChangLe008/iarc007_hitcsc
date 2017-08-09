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
time = '8月3日23时38'
str_a = '/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/control.txt'
str_b = '/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/height.txt'
str_c = '/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/velocity.txt'
a = open(str_a)
b = open(str_b)#7月6日20时57
c = open(str_c)#7月11日22时44
text = a.readlines()
test = b.readlines()
tedt = c.readlines()


c_m = []
s_18 = []
s_45 = []
as_m = []
att_c = []
roll = []
pitch = []
yaw = []
for ctdata in text:
    ct = ctdata.split("\t")
    c_m.append(string.atof(ct[0]))
    s_18.append(string.atof(ct[1]))
    s_45.append(string.atof(ct[2]))
    as_m.append(string.atof(ct[3]))
    roll.append(string.atof(ct[4]))
    pitch.append(string.atof(ct[5]))
    yaw.append(string.atof(ct[6]))

t_op = []

ttt = []
dx = []
dy = []
dz = []
Xkf = []
Vkf = []
Xkf1 = []
Vkf1 = []
dirx_t = []
diry_t = []
t_n     = []    #target_number
t_c     = []    #choosed_target
xo  = []
yo  = []
zo  = []
ax1  = []
ay1 = []
az1  = []
vx1  = []
vy1  = []
vz1  = []
tttx = []
ttty = []
compens_vx = []
compens_vy = []
compens = []

tar_xkf = []
tar_ykf = []
for data in tedt:
    d = data.split("\t")
    ttt.append(string.atof(d[0]))
    t_op.append(string.atof(d[0])+0.2)
    t_n.append(string.atof(d[1]))
    t_c.append(string.atof(d[2]))
    dx.append(string.atof(d[3]))
    dy.append(string.atof(d[4]))
    Xkf.append(string.atof(d[5]))
    Vkf.append(string.atof(d[6]))
    Xkf1.append(string.atof(d[7]))
    Vkf1.append(0.01+string.atof(d[8]))
    dirx_t.append(string.atof(d[9]))
    diry_t.append(string.atof(d[10]))
    xo.append(string.atof(d[11]))
    yo.append(string.atof(d[12]))
    zo.append(string.atof(d[13]))
    dz.append(string.atof(d[14]))
    ax1.append(string.atof(d[15]))
    ay1.append(string.atof(d[16]))
    az1.append(string.atof(d[17]))
    vx1.append(string.atof(d[18]))
    vy1.append(string.atof(d[19]))
    vz1.append(string.atof(d[20]))
    compens_vx.append(string.atof(d[21]))
    compens_vy.append(string.atof(d[22]))
    compens.append(math.sqrt(string.atof(d[21])*string.atof(d[21])+string.atof(d[22])*string.atof(d[22])))
    tttx.append(string.atof(d[18])-string.atof(d[6]))
    ttty.append(string.atof(d[19])-string.atof(d[8]))
    tar_xkf.append(string.atof(d[23]))
    tar_ykf.append(string.atof(d[24]))

th  = []
ax  = []
ay  = []
az  = []
vx  = []
vy  = []
vz  = []
lpx = []
lpy = []
lpz = []
gpa = []
gph = []
ul  = []
opx = []
opy = []
opz = []
omg = []
target_vx = []
target_vy = []
filt = []
wx = []
wy = []
wz = []
for data in test:
    d = data.split("\t")
    print len(d)
    th.append(string.atof(d[0]))
    ax.append(math.sqrt(string.atof(d[1])*string.atof(d[1])+string.atof(d[2])*string.atof(d[2])))
    ay.append(string.atof(d[2]))
    az.append(string.atof(d[3]))
    vx.append(string.atof(d[4]))
    vy.append(string.atof(d[5]))
    vz.append(string.atof(d[6]))
    lpx.append(string.atof(d[7]))
    lpy.append(string.atof(d[8]))
    lpz.append(string.atof(d[9]))
    gpa.append(string.atof(d[10]))
    gph.append(string.atof(d[11]))
    ul.append(string.atof(d[12]))
    opx.append(-1*string.atof(d[13])-1.04)
    opy.append(string.atof(d[14])-yo[0]-0.24)
    opz.append(string.atof(d[15]))
    target_vx.append(string.atof(d[18]))
    target_vy.append(string.atof(d[19]))
    omg.append(string.atof(d[17]))
    filt.append(string.atof(d[20]))
    wx.append(string.atof(d[21]))
    wy.append(string.atof(d[22]))
    wz.append(string.atof(d[23]))

fig = plt.figure(2)
plt.plot(ttt,t_n,'g--')
plt.plot(ttt,dx,'r')
plt.plot(ttt,dy,'b')
plt.plot(ttt,filt,'b--')
# plt.plot(ttt,opy,'r--')
plt.legend()

# fig = plt.figure(2)
# plt.plot(ttt,target_vx,'b',label='target_vx')
# plt.plot(ttt,Vkf,'r',label='Vkf')
# plt.legend()

# fig = plt.figure(3)
# plt.plot(ttt,target_vy,'b',label='target_vy')
# plt.plot(ttt,Vkf1,'r',label='Vkf1')
# plt.legend()

# fig = plt.figure(4)
# # plt.plot(ttt,tttx,'b',label='tttx')
# # plt.plot(ttt,ttty,'r',label='ttty')
# plt.plot(ttt,compens_vx,'y',label='compens_vx')
# plt.plot(ttt,compens_vy,'g',label='compens_vy')
# plt.plot(ttt,compens,'r',label='compens')
# plt.legend()

# for al in range(len(dy)):
#     dx[al] = dx[al]-tar_xkf[al]
# for al in range(len(dy)):
#     dy[al] = dy[al]-tar_ykf[al]

fig = plt.figure(6)
# plt.plot(ttt,dx,'b',label='dx')
# plt.plot(ttt,dy,'r',label='dy')
# plt.plot(ttt,Vkf,'k',label='Vkf')
# plt.plot(ttt,Vkf1,'m',label='Vkf1')
# # plt.plot(ttt,dz,'g',label='dz')dirx_t
plt.plot(ttt,dirx_t,'k',label='xdirt')
plt.plot(ttt,diry_t,'m',label='ydir')
plt.plot(ttt,tar_xkf,'r2',label='x')
plt.plot(ttt,tar_ykf,'g2',label='y')
plt.plot(ttt,s_45,'k',label='stage')

# plt.plot(ttt,t_n,'g--',label='tn')
plt.legend()

fig = plt.figure(1)
plt.xlabel("time/s")
plt.plot(ttt,s_18,'g',label='s_18')

al_temp = gpa[0]
for al in range(len(gpa)):
    gpa[al] = gpa[al]-al_temp+0.173

al_temp = omg[0]
for al in range(len(omg)):
    omg[al] = omg[al]-al_temp

al_temp = lpz[0]
for al in range(len(lpz)):
    lpz[al] = lpz[al]-al_temp+0.173
al_temp = opz[0]
for al in range(len(opz)):
    opz[al] = opz[al]-al_temp-0.075
# plt.plot(ttt,gph,'g2',label='gph')
# plt.plot(ttt,ul,'k',label='ultasonic')
# plt.plot(ttt,omg,'g',label='fused height')

plt.plot(ttt,filt,'r',label='gpa')
# plt.plot(ttt,lpz,'c',label='lpz')
# plt.plot(t_op,opz,'g',label='optitrack')
plt.plot(t_op,az,'g',label='acc')
# plt.plot(t_op,wx,'r',label='wx')
# plt.plot(t_op,wy,'b',label='wy')
plt.plot(ttt,s_45,'k',label='stage')
plt.plot(ttt,ul,'k',label='ul')
# # fig = plt.figure(2)
# plt.plot(ttt,vz)
# plt.plot(ttt,pitch,label='pitch')
# plt.plot(ttt,roll,label='roll')
plt.title('height test')
plt.xlabel("time/s")
plt.ylabel("height/m")


"""

data mining

"""

plt.legend()
plt.grid()
plt.show()
