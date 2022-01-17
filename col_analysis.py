from scipy.interpolate import CubicSpline
import sys
import numpy as np
import matplotlib.pyplot as plt

with open("data/10.txt","r") as fp:
    str1 = fp.read()

each_line = str1.strip().split("\n")
xaxis = []
temp  = []
heat  =[]
sync_flg = False
for line in each_line:
    data = line.split(" ")
    if (len(data) == 3):
        if(int(data[2])== 100):
            sync_flg = True
        if(sync_flg):
            xaxis.append(int(data[0]))
            temp.append(float(data[1]))
            heat.append(int(data[2]))
print(max(np.diff(xaxis)))
#======================================
with open("data/20.txt","r") as fp:
    str1 = fp.read()
each_line = str1.strip().split("\n")

xaxis2 = []
temp2 = []
heat2 = []
sync_flg = False
for line in each_line:
    data = line.split(" ")
    if (len(data) == 3):
        if(int(data[2])== 100):
            sync_flg = True
        if(sync_flg):
            xaxis2.append(int(data[0]))
            temp2.append(float(data[1]))
            heat2.append(int(data[2]))
print(max(np.diff(xaxis2)))
#======================================
with open("data/40.txt","r") as fp:
    str1 = fp.read()
each_line = str1.strip().split("\n")

xaxis3 = []
temp3 = []
heat3 = []
sync_flg = False
for line in each_line:
    data = line.split(" ")
    if (len(data) == 3):
        if(int(data[2])== 100):
            sync_flg = True
        if(sync_flg):
            xaxis3.append(int(data[0]))
            temp3.append(float(data[1]))
            heat3.append(int(data[2]))
print(max(np.diff(xaxis3)))
#======================================
with open("data/60.txt","r") as fp:
    str1 = fp.read()
each_line = str1.strip().split("\n")

xaxis4 = []
temp4 = []
heat4 = []
sync_flg = False
for line in each_line:
    data = line.split(" ")
    if (len(data) == 3):
        if(int(data[2])== 100):
            sync_flg = True
        if(sync_flg):
            xaxis4.append(int(data[0]))
            temp4.append(float(data[1]))
            heat4.append(int(data[2]))
print(max(np.diff(xaxis4)))

#======================================
with open("data/80.txt","r") as fp:
    str1 = fp.read()
each_line = str1.strip().split("\n")

xaxis5 = []
temp5 = []
heat5 = []
sync_flg = False
for line in each_line:
    data = line.split(" ")
    if (len(data) == 3):
        if(int(data[2])== 100):
            sync_flg = True
        if(sync_flg):
            xaxis5.append(int(data[0]))
            temp5.append(float(data[1]))
            heat5.append(int(data[2]))
print(max(np.diff(xaxis5)))
# Normalize x axis
length = min([len(xaxis),len(xaxis2),len(xaxis3),len(xaxis4),len(xaxis5)])
norm_xaxis = [17*x for x in range(0,length)]

plt.plot(norm_xaxis, temp[:length], label="10 uL/min")
plt.plot(norm_xaxis, temp2[:length], label="20 uL/min")
plt.plot(norm_xaxis, temp3[:length], label="40 uL/min")
plt.plot(norm_xaxis, temp4[:length], label="60 uL/min")
plt.plot(norm_xaxis, temp5[:length], label="80 uL/min")
plt.plot(norm_xaxis, heat[:length], label="heat")

plt.legend(loc="upper left")
plt.show()
