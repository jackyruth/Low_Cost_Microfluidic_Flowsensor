from scipy.interpolate import CubicSpline
import sys
import numpy as np
import matplotlib.pyplot as plt

x_time = []
temp   = []
heater = []

first_heater = False
old_time = 0
for line in sys.stdin:
    data = line.strip().split(' ')
    if((len(data) == 3)and(int(data[0])>old_time)):
        if(int(data[2])== 100):
            first_heater = True
        if(first_heater):
            old_time =int(data[0])
            x_time.append(int(data[0]))
            temp.append(float(data[1]))
            heater.append(int(data[2]))
    else:
        print(data)

plt.plot(x_time, temp, label = "temp")
plt.plot(x_time, heater, label = "heater")
plt.show()



'''
no_timestamp = "500uL_per_min_100ms.txt"
# no_timestamp = "1.txt"
with open(no_timestamp, "r") as fp:
    data = fp.readlines()
x_axis = []
y_axis = []
heater = []
tick = 0
for item in data:
    if (int(item.rstrip().split(" ")[1])==100):
        heater.append(25)
    else:
        heater.append(24)
    y_axis.append(float(item.rstrip().split(" ")[0]))
    tick += 1
    x_axis.append(tick)
plt.plot(y_axis, label = "temp")
plt.plot(heater, label = "heater")

f=CubicSpline(x_axis,y_axis, bc_type='natural')
x_new = np.linspace(0,x_axis[-1],(x_axis[-1]+1)//20)
y_new = f(x_new)
plt.plot(x_new, y_new, label = "heater")
plt.show()
with open("10.txt", "r") as fp:
    data = fp.readlines()

x_axis = []
y_axis = []
heater = []
normalize = int(data[0].rstrip().split(" ")[0])
for item in data:
    if (int(item.rstrip().split(" ")[2])==100):
        heater.append(25)
    else:
        heater.append(24)
    y_axis.append(float(item.rstrip().split(" ")[1]))
    x_axis.append(int(item.rstrip().split(" ")[0])-normalize)
plt.plot(x_axis, y_axis, label = "temp")
plt.plot(x_axis, heater, label = "heater")
f=CubicSpline(x_axis,y_axis, bc_type='natural')
x_new = np.linspace(0,x_axis[-1],(x_axis[-1]+1)//1000)
y_new = f(x_new)
plt.plot(x_new, y_new, label = "heater")
plt.show()
'''
