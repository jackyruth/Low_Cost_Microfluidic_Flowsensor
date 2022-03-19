# Julia
The tool we use for data analysis is julia and Pluto notebook.

As mentioned in the key documents, in order to extract the flow rate from the time series temperature data, we have to go through two steps.
1. Extract the timedelta from the time series temperature data. This process is shown in 'timedelta_detection.jl'
2. Extract the flowrate from the timedelta. This process is shown in 'flowrate_detection.jl'

We also use the same tools to analyze the benchmarking data. This process is shown in 'benchmarking.jl'
