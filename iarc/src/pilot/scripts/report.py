#coding:UTF-8

"""
Created on 2017/04/07
@author: Leonidas
"""
import matplotlib.pyplot as plt
import string
import reportlab.lib.fonts
from reportlab.pdfgen.canvas import Canvas
from reportlab.lib.units import inch,cm
from reportlab.platypus import Table,Paragraph,Spacer,Image
from reportlab.lib.pagesizes import A4, landscape  
from PIL import Image

def pdf_head(canvas, headtext):
    #setFont是字体设置的函数，第一个参数是类型，第二个是大小
    canvas.setFont("Helvetica-Bold", 11.5)  
    #向一张pdf页面上写string
    canvas.drawString(1*inch, 10.5*inch, headtext)  
    #画一个矩形，并填充为黑色
    canvas.rect(1*inch, 10.3*inch, 6.5*inch, 0.12*inch,fill=1) 
    #画一条直线
    canvas.line(1*inch, 10*inch, 7.5*inch, 10*inch) 


#ResultFormat = "%0.4f"
#Usage: ResultFormat%(1/3)
time = '7月23日11时41'
str_a = '/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/control.txt'
str_b = '/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/height.txt'
str_c = '/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/velocity.txt'
report = '/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/report.pdf'

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
    # dz.append(string.atof(d[14]))
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

for data in test:
    d = data.split("\t")
    th.append(string.atof(d[0]))
    ax.append(string.atof(d[1]))
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
    # target_vx.append(string.atof(d[18]))
    # target_vy.append(string.atof(d[19]))
    omg.append(string.atof(d[-1]))

# fig = plt.figure(2)
# plt.plot(ttt,t_n,'g--')
# plt.plot(ttt,dx,'r')
# plt.plot(ttt,dy,'b')
# plt.plot(ttt,opx,'b--')
# plt.plot(ttt,opy,'r--')

# fig = plt.figure(2)
# plt.plot(ttt,target_vx,'b',label='target_vx')
# plt.plot(ttt,Vkf,'r',label='Vkf')
# plt.legend()

# fig = plt.figure(3)
# plt.plot(ttt,target_vy,'b',label='target_vy')
# plt.plot(ttt,Vkf1,'r',label='Vkf1')
# plt.legend()

fig = plt.figure(6,figsize=(8,6))
plt.plot(ttt,dx,'b',label='dx')
plt.plot(ttt,dy,'r',label='dy')
# plt.plot(ttt,dz,'g',label='dz')
plt.plot(ttt,t_n,'g--',label='tn')
plt.legend()

fig = plt.figure(1)
plt.xlabel("time/s")
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
plt.plot(ttt,ul,'k',label='ultasonic')
plt.plot(ttt,omg,'r',label='fused height')

# # plt.plot(ttt,gpa,'r',label='gpa')
# # plt.plot(ttt,lpz,'c',label='lpz')
plt.plot(t_op,opz,'g',label='optitrack')
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
plt.savefig('/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/1.eps',dpi=600)
plt.show()


can = Canvas(report)
pdf_head(can,"test for report lab!!!")
can.showPage()
# Image('/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/1.svg',1000,1000)
(h,w) = landscape(A4)


# Image_file = open('/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/1.svg')
# image_width, image_height = Image_file.size
# image_aspect = image_height / float(image_width)
# Determine the dimensions of the image in the overview
print_width = w
print_height = w * 3/4

can.drawImage('/home/hitcsc/catkin_ws/log/iarc/pilot/'+time+'/1.eps',w - print_width, h - print_height, width=print_width,height=print_height) 
can.showPage()
can.save()