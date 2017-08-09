import numpy as np
import string


num = input("Enter the number")
l_s = raw_input("enter the location")
l_o = l_s.split(",")
location = [string.atof(l_o[0]),string.atof(l_o[1])]
store = []
for i in range(num):
    temp = raw_input("enter the data")
    data = temp.split(",")
    ttemp = []
    for j in range(len(data)):
        ttemp.append(string.atof(data[j]))
    store.append(ttemp)

print store