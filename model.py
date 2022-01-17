import math
import matplotlib.pyplot as plt

delta_t = 17

# units are W, mm, s, K
thermal_conductivity = 0.598/1000 # W/mm*K @ 20 degrees
thermal_diffusivity  = 0.143/1000 # mm^2/s @ 25 degrees
signal_strength      = 38
distance_from_heater = 2 # mm
speed                = 0.83 # mm/s

def funk(t, q, v):
    k = thermal_conductivity
    a = thermal_diffusivity
    # q = signal_strength
    x = distance_from_heater
    return (q/(4*math.pi*k*t)*math.e**(-(x-v*t)**2/(4*a*t)))


if (__name__ == "__main__"):
    t_list = [delta_t*i for i in range(1,2000)]
    rslt = []
    for t in t_list: 
        rslt.append(funk(t, 50, 0.0002))
    plt.plot(t_list,rslt)
    plt.show()
