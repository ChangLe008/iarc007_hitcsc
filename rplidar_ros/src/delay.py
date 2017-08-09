import matplotlib.pyplot as plt
import math
import string
ResultFormat = "%0.4f"
#Usage: ResultFormat%(1/3)
a = open('/home/hitcsc/catkin_ws/log/test_client/puredelay.txt')
text = a.readlines();
h=[];
t=[];
for b in text:
    c = b.split("\t");
    #print c;
    h.append(string.atof(c[0]));
    t.append(string.atof(c[1]));

fig = plt.figure(1)
plt.plot(t,h,'b');
d = sum(h)/len(h)
plt.plot([t[0],t[-1]],[d,d],'g',linewidth=2.0)
title = "The average of transport delay is: "+str("%0.6f"%(d))
plt.title(title)
plt.grid();
plt.show();