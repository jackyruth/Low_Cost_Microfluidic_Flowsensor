# Data
Data comes in the following form
```
158781 26.30 0
158801 26.30 0
158821 26.31 0
158841 26.30 100
158861 26.30 100
158881 26.29 100
158901 26.29 100
158921 26.30 100
158941 26.28 100
158961 26.28 0
158981 26.28 0
```
The first column is time in milliseconds.
The second column is temperature data in Celsius
The third column is the heat pulse signal. When it is 0, the heater is off, when it is 100, the heater is on.

The data is stored in The data is stored in text files named with the corresponding flowrate.
These text files are stored in repositiories corresponding to the device used to take those measurements.

For example 'final_product/10.txt' contain time series temperature data taken by the final product at a set flow rate of 10 uL/min
