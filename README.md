# Low Cost Flowsensor
https://github.com/jackyruth/Low_Cost_Microfluidic_Flowsensor/assets/32418878/62e07f6d-2dad-415f-953b-cadd57041e67

## Description
- Does the Job! Measures up to 80 uL/min
- Cheap! Has a BOM cost < 30CAD
- Accurate! 5% Median Accuracy < 55 uL/min, 10-18% Median Accuracy > 55uL/min
- Non-invasive! Wet components are standard 1/32' OD tygon tubing and 29G stainless steel
- Simple Interface! UART Outputs every 15 seconds

This project is also well documented:
- 'Flowsensor Product Specification.pdf': Datasheet for the flowsensor
- 'Flowsensor Requirements, Design & Verification.pdf': A document describing the requirements, design and verification of the flowsensor
- 'flowsensor_demonstration.MOV': A video showing the benchmarking process
- 'UBC_BioMEMS_Product_Video.mp4': A video describing the context of the system the flowsensor is originally designed to be a part of.


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
