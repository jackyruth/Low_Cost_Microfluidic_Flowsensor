# Low Cost Flowsensor
## Description
- Does the Job! Measures from up to 80 uL/min
- Cheap! Has a BOM cost of < 30CAD
- Accurate! 5% Median Accuracy < 55 uL/min, 10-18% Median Accuracy > 55uL/min
- Non-invasive! Wet components are standard 1/32' OD tygon tubing and 29G stainless steel
- Simple Interface! UART Outputs every 15 seconds


## Project Tree
```
.
├── data
├── julia
├── lsi_lab_validation
├── pcb_v1
├── README.md
├── reagent_compatibility
└── stm32
```

- 'data' contains time series temperature data measured at set flow rates
- 'julia' contains data science Pluto notebooks used to experiment with the time series temperature data and analyze benchmarking data 
- 'lsi_lab_validation' contains the benchmarking data where we pitted the low cost sensor against the Sensirion SLI-0430 FMK flowsensor
- 'pcb_v1' contains a KiCAD PCB project of the flowsensor
- 'reagent_compatibility' contains benchmarking data where we ran different reagents, some at different temperatures, through the flowsensor
- 'stm32' contains the firmware for the flowsensor
