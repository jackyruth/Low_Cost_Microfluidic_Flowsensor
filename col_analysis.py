import yaml
import numpy as np
import matplotlib.pyplot as plt

normalize = False

with open("data_to_analyze.yml", 'r') as fp:
    data_loaded = yaml.safe_load(fp)
path = data_loaded["data_path"]
filenames = data_loaded["data_files"]

length_list = []
temp_list = []
for filename in filenames:
    min_temp = 100
    with open(path+filename,"r") as fp:
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
    print(max(np.diff(xaxis)))
    length_list.append(len(xaxis))
    temp_list.append(temp)

# Normalize x axis
length = min(length_list)
norm_xaxis = [17*x for x in range(0,length)]

idx = 0
for ele in temp_list:
    plt.scatter(norm_xaxis, ele[:length], label=filenames[idx],s=2)
    idx += 1

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
