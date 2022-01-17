from scipy.interpolate import CubicSpline
import sys
import numpy as np
import matplotlib.pyplot as plt

min_temp = 100
normalize = False

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
if(normalize): 
    for ele in temp:
        if(ele < min_temp):
            min_temp = ele
    for i in range(len(temp)):
        temp[i] -= min_temp
min_temp = 100
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
if(normalize): 
    for ele in temp2:
        if(ele < min_temp):
            min_temp = ele
    for i in range(len(temp2)):
        temp2[i] -= min_temp
min_temp = 100
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
if(normalize): 
    for ele in temp3:
        if(ele < min_temp):
            min_temp = ele
    for i in range(len(temp3)):
        temp3[i] -= min_temp
min_temp = 100
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
if(normalize): 
    for ele in temp4:
        if(ele < min_temp):
            min_temp = ele
    for i in range(len(temp4)):
        temp4[i] -= min_temp
min_temp = 100
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
if(normalize): 
    for ele in temp5:
        if(ele < min_temp):
            min_temp = ele
    for i in range(len(temp5)):
        temp5[i] -= min_temp
print(max(np.diff(xaxis5)))


# Normalize x axis
length = min([len(xaxis),len(xaxis2),len(xaxis3),len(xaxis4),len(xaxis5)])
norm_xaxis = [17*x for x in range(0,length)]

plt.scatter(norm_xaxis, temp[:length], label="10 uL/min",s=2)
plt.scatter(norm_xaxis, temp2[:length], label="20 uL/min",s=2)
plt.scatter(norm_xaxis, temp3[:length], label="40 uL/min",s=2)
plt.scatter(norm_xaxis, temp4[:length], label="60 uL/min",s=2)
plt.scatter(norm_xaxis, temp5[:length], label="80 uL/min",s=2)
plt.plot(norm_xaxis, heat[:length], label="heat")
plt.legend(loc="upper right")
plt.xlabel("Time (ms)")
plt.title("temperature vs time for various flowrates")
if(normalize): 
    plt.ylim(0,3)
    plt.ylabel("Temperature Delta (Celsius)")
else:
    plt.ylim(20,26)
    plt.ylabel("Temperature (Celsius)")
plt.show()
