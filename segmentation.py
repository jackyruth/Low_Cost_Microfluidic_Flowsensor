import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
import model as md

segment_length = 10000 # milliseconds
delta_t = 17 # milliseconds
max_delta_t = 18 # milliseconds
flowrate_data = ["10","20","40","60","80"]
for flowrate in flowrate_data:
    with open("data/"+flowrate+".txt","r") as fp:
        datalist = fp.read().strip().split("\n")

    sync = False
    segment_list = []
    
    for ele in datalist:
        ele_list = ele.split(" ")
        if(len(ele_list)==3): #occasionally missing data
            time = int(ele_list[0])
            temp = float(ele_list[1])
            heat = int(ele_list[2])
            if((heat==100)and(sync==False)):
                sync = True
                new_segment = {"time":[],"temp":[],"heat":[]}
                idx = 0
    
            if(sync):
                new_segment["time"].append(time)
                new_segment["temp"].append(temp)
                new_segment["heat"].append(heat)
                idx += 1
                if(idx >= segment_length//delta_t):
                    segment_list.append(new_segment)
                    sync = False
    
    # Segment filtering
    segment_list = segment_list [:-1] # remove last segment as it may be incomplete
    segments= [] 
    for seg in segment_list:
        if(max(np.diff(seg["time"]))<max_delta_t):
            time_len = len(seg["time"])
            if(len(seg["temp"]) == time_len):
                if(len(seg["heat"]) == time_len):
                    segments.append(seg)
    
    # Info Printouts
    print("Number of segments: " + str(len(segments)))
    idx = 0
    for seg in segments:
        idx += 1
        print("Segment "+str(idx)+": " + str(len(seg["time"])*delta_t) +"ms")
    
    
    # Segment overlay plot
    max_temp = 0
    min_temp = 100
    seg_num = 1
    for seg in segments:
        length = len(seg["time"])
        norm_axis = [delta_t*(x+1) for x in range(0,length)]
        if(max(seg["temp"]) > max_temp):
            max_temp = max(seg["temp"])
        if(min(seg["temp"]) < min_temp):
            min_temp = min(seg["temp"])
        max_temp = max_temp - min_temp
        norm_temp = []
        for temp in seg["temp"]:
            norm_temp.append(temp - min_temp)
    
        ''' Modeling
        popt, pcov = curve_fit(md.funk,norm_axis,norm_temp[:length])
        rslt = []
        for t in norm_axis:
            rslt.append(md.funk(t/1000, popt[0],popt[1])/1000)
        plt.plot(norm_axis, rslt)
        '''
        plt.scatter(norm_axis, norm_temp[:length], s=2,label="trial "+str(seg_num))
        plt.plot(norm_axis, seg["heat"][:length])
        seg_num += 1
    plt.legend(loc="upper right")
    plt.xlabel("Time (ms)")
    plt.ylabel("Temperature Delta (Celsius)")
    plt.title(flowrate+" uL/min")
    plt.ylim(0,max_temp)
    plt.show()
