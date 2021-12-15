# STDIN
Degrees celcius seperated by newline

23.953125
23.960937
23.960937

# Realtime Plotting
https://github.com/tenox7/ttyplot

py -m serial /dev/ttyACM0 115200 | ttyplot -m 25 -M 23 -t "Temperature Graph" -u oC
