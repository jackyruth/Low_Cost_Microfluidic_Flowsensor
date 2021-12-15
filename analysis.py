from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt
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
'''
with open("500uL_per_min.txt", "r") as fp:
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
