# STM32
There are two firmware files.

The 'data_collection' firmware is used to collect timeseries temperature data.
Applying the 'flowrate_detection.jl' on that temperature data will yield the calibration constants.

The 'functional_firmware' is the main firmware which runs on the sensor and outputs the flowrate. The calibration constants are hardcoded as #define in this firmware.
