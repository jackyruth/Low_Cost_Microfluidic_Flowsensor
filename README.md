# Low Cost Flowsensor

## Project Layout
├── data
├── julia
├── pcb_v1
├── README.md
└── stm32

### data
Contains temperature measurement data for various sensor prototypes
**pcb_data**: Data from the pcb sensor
**prototype_2mm**: Sensor hacked together from Sparkfun Sensor Board. Distance between heater and sensor is aroudn 2mm
**prototype_5mm**: Sensor hacked together from Sparkfun Sensor Board. Distance between heater and sensor is aroudn 5mm
**misc**: Data from unknown prototypes

### julia
Contains Pluto notebooks which perform the data science analysis
**notebook.jl** Pluto notebook that analysis temperature pulses and extract the timedelta
**statistical_analysis.jl** Pluto notebook that compares extracted timedelta against flowrate. Also fits an exponential model for the flowrate-timedelta curve

### pcb_v1
Flowsensor PCB KiCAD Project

### stm32
The calibration and functional firmware
**data_collection**: Used to collect temperature data. Program this firmware and save the temperature data in .txt files. Then import the data into **statistical_analysis.jl** in julia and find the configuration parameters A, B and C. Finally, hardcode the constants in **functional_firmware**

**functional_firmware**: Outputs a flowrate every 15 seconds

```mermaid
flowchart TD
    HeatPulse(100 ms Heat Pulse)
    DataCollection(Collect Temperature Data)
    Sleep(Wait for 1 second)
    ArrayFull([Are there 13 data points?])
    PassThreshold([Is temperature bump significant?])
    NoFlow[[Output 0.00 uL/min]]
    LM_Algo(Solve Non-Linear Least Squares Problem)
    ConvergenceCheck([Did solution converge?])
    OOR[[Output OOR for out of range]]
    Get_Timedelta(Extract Timedelta)
    Get_Flowrate(Calculate Flow Rate)
    Flowrate_Result[[Output Flow Rate]]
	Start --> HeatPulse
	HeatPulse --> DataCollection
	DataCollection --> ArrayFull
	ArrayFull -- no -->Sleep
	Sleep --> DataCollection
	ArrayFull -- yes -->PassThreshold
	PassThreshold -- no --> NoFlow
	PassThreshold -- yes --> LM_Algo
	LM_Algo --> ConvergenceCheck
	ConvergenceCheck -- no --> OOR
	ConvergenceCheck -- yes --> Get_Timedelta
	Get_Timedelta --> Get_Flowrate
	Get_Flowrate --> Flowrate_Result
```
