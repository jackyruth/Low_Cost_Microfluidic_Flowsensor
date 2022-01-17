EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Low Cost Flowsensor"
Date "2022-01-15"
Rev "1"
Comp "UBC BioMEMS"
Comment1 "Flow range: 0 - 80 uL/min "
Comment2 "Author: Jacky Jiang"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L flowsensor:tmp117 U1
U 1 1 61E12858
P 2250 2850
F 0 "U1" H 2525 3515 50  0000 C CNN
F 1 "tmp117" H 2525 3424 50  0000 C CNN
F 2 "Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm" H 2150 3050 50  0001 C CNN
F 3 "" H 2150 3050 50  0001 C CNN
	1    2250 2850
	1    0    0    -1  
$EndComp
Text Label 2150 2450 2    50   ~ 0
SCL
Text Label 2150 3250 2    50   ~ 0
SCL
Text Label 2150 4050 2    50   ~ 0
SCL
Text Label 2900 2450 0    50   ~ 0
SDA
Text Label 2900 3250 0    50   ~ 0
SDA
Text Label 2900 4050 0    50   ~ 0
SDA
Text Label 5100 3950 3    50   ~ 0
SDA
Text Label 5000 3950 3    50   ~ 0
SCL
$Comp
L Device:R R1
U 1 1 61E15684
P 5000 3800
F 0 "R1" H 5150 3750 50  0000 C CNN
F 1 "2.2k" H 5150 3850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" V 4930 3800 50  0001 C CNN
F 3 "~" H 5000 3800 50  0001 C CNN
	1    5000 3800
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 61E16851
P 5100 3800
F 0 "R2" H 4950 3750 50  0000 C CNN
F 1 "2.2k" H 4950 3850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" V 5030 3800 50  0001 C CNN
F 3 "~" H 5100 3800 50  0001 C CNN
	1    5100 3800
	-1   0    0    1   
$EndComp
$Comp
L power:VDD #PWR0101
U 1 1 61E17860
P 5050 3650
F 0 "#PWR0101" H 5050 3500 50  0001 C CNN
F 1 "VDD" H 5067 3823 50  0000 C CNN
F 2 "" H 5050 3650 50  0001 C CNN
F 3 "" H 5050 3650 50  0001 C CNN
	1    5050 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 3650 5050 3650
Connection ~ 5050 3650
Wire Wire Line
	5050 3650 5100 3650
$Comp
L power:VDD #PWR0102
U 1 1 61E1D23D
P 3300 2550
F 0 "#PWR0102" H 3300 2400 50  0001 C CNN
F 1 "VDD" H 3317 2723 50  0000 C CNN
F 2 "" H 3300 2550 50  0001 C CNN
F 3 "" H 3300 2550 50  0001 C CNN
	1    3300 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 61E1E58B
P 2150 4150
F 0 "#PWR0105" H 2150 3900 50  0001 C CNN
F 1 "GND" V 2155 4022 50  0000 R CNN
F 2 "" H 2150 4150 50  0001 C CNN
F 3 "" H 2150 4150 50  0001 C CNN
	1    2150 4150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 61E1EAF7
P 2150 3350
F 0 "#PWR0106" H 2150 3100 50  0001 C CNN
F 1 "GND" V 2155 3222 50  0000 R CNN
F 2 "" H 2150 3350 50  0001 C CNN
F 3 "" H 2150 3350 50  0001 C CNN
	1    2150 3350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 61E1EF5E
P 2150 2550
F 0 "#PWR0107" H 2150 2300 50  0001 C CNN
F 1 "GND" V 2155 2422 50  0000 R CNN
F 2 "" H 2150 2550 50  0001 C CNN
F 3 "" H 2150 2550 50  0001 C CNN
	1    2150 2550
	0    1    1    0   
$EndComp
$Comp
L flowsensor:tmp117 U3
U 1 1 61E22BB4
P 2250 4450
F 0 "U3" H 2525 5115 50  0000 C CNN
F 1 "tmp117" H 2525 5024 50  0000 C CNN
F 2 "Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm" H 2150 4650 50  0001 C CNN
F 3 "" H 2150 4650 50  0001 C CNN
	1    2250 4450
	1    0    0    -1  
$EndComp
$Comp
L flowsensor:tmp117 U2
U 1 1 61E22836
P 2250 3650
F 0 "U2" H 2525 4315 50  0000 C CNN
F 1 "tmp117" H 2525 4224 50  0000 C CNN
F 2 "Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm" H 2150 3850 50  0001 C CNN
F 3 "" H 2150 3850 50  0001 C CNN
	1    2250 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 61E299F8
P 2000 2650
F 0 "R3" H 2150 2600 50  0000 C CNN
F 1 "10k" H 2150 2700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" V 1930 2650 50  0001 C CNN
F 3 "~" H 2000 2650 50  0001 C CNN
	1    2000 2650
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 61E29FB8
P 2000 3450
F 0 "R4" H 2150 3400 50  0000 C CNN
F 1 "10k" H 2150 3500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" V 1930 3450 50  0001 C CNN
F 3 "~" H 2000 3450 50  0001 C CNN
	1    2000 3450
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 61E2A932
P 2000 4250
F 0 "R5" H 2150 4200 50  0000 C CNN
F 1 "10k" H 2150 4300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" V 1930 4250 50  0001 C CNN
F 3 "~" H 2000 4250 50  0001 C CNN
	1    2000 4250
	0    1    1    0   
$EndComp
$Comp
L flowsensor:tmp117 U4
U 1 1 61E2B640
P 2250 5250
F 0 "U4" H 2525 5915 50  0000 C CNN
F 1 "tmp117" H 2525 5824 50  0000 C CNN
F 2 "Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm" H 2150 5450 50  0001 C CNN
F 3 "" H 2150 5450 50  0001 C CNN
	1    2250 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 61E2C169
P 2000 5050
F 0 "R6" H 2150 5000 50  0000 C CNN
F 1 "10k" H 2150 5100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" V 1930 5050 50  0001 C CNN
F 3 "~" H 2000 5050 50  0001 C CNN
	1    2000 5050
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR01
U 1 1 61E2CFCA
P 1850 2650
F 0 "#PWR01" H 1850 2500 50  0001 C CNN
F 1 "VDD" V 1868 2777 50  0000 L CNN
F 2 "" H 1850 2650 50  0001 C CNN
F 3 "" H 1850 2650 50  0001 C CNN
	1    1850 2650
	0    -1   -1   0   
$EndComp
$Comp
L power:VDD #PWR02
U 1 1 61E2D324
P 1850 3450
F 0 "#PWR02" H 1850 3300 50  0001 C CNN
F 1 "VDD" V 1868 3577 50  0000 L CNN
F 2 "" H 1850 3450 50  0001 C CNN
F 3 "" H 1850 3450 50  0001 C CNN
	1    1850 3450
	0    -1   -1   0   
$EndComp
$Comp
L power:VDD #PWR03
U 1 1 61E2DF42
P 1850 4250
F 0 "#PWR03" H 1850 4100 50  0001 C CNN
F 1 "VDD" V 1868 4377 50  0000 L CNN
F 2 "" H 1850 4250 50  0001 C CNN
F 3 "" H 1850 4250 50  0001 C CNN
	1    1850 4250
	0    -1   -1   0   
$EndComp
$Comp
L power:VDD #PWR04
U 1 1 61E2E93E
P 1850 5050
F 0 "#PWR04" H 1850 4900 50  0001 C CNN
F 1 "VDD" V 1868 5177 50  0000 L CNN
F 2 "" H 1850 5050 50  0001 C CNN
F 3 "" H 1850 5050 50  0001 C CNN
	1    1850 5050
	0    -1   -1   0   
$EndComp
Text Label 2150 4850 2    50   ~ 0
SCL
$Comp
L power:GND #PWR05
U 1 1 61E2FA92
P 2150 4950
F 0 "#PWR05" H 2150 4700 50  0001 C CNN
F 1 "GND" V 2155 4822 50  0000 R CNN
F 2 "" H 2150 4950 50  0001 C CNN
F 3 "" H 2150 4950 50  0001 C CNN
	1    2150 4950
	0    1    1    0   
$EndComp
Text Label 2900 4850 0    50   ~ 0
SDA
$Comp
L power:VDD #PWR06
U 1 1 61E30D4E
P 2900 2650
F 0 "#PWR06" H 2900 2500 50  0001 C CNN
F 1 "VDD" V 2917 2778 50  0000 L CNN
F 2 "" H 2900 2650 50  0001 C CNN
F 3 "" H 2900 2650 50  0001 C CNN
	1    2900 2650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 61E3115E
P 2900 3450
F 0 "#PWR07" H 2900 3200 50  0001 C CNN
F 1 "GND" V 2905 3322 50  0000 R CNN
F 2 "" H 2900 3450 50  0001 C CNN
F 3 "" H 2900 3450 50  0001 C CNN
	1    2900 3450
	0    -1   -1   0   
$EndComp
Text Label 2900 4250 0    50   ~ 0
SDA
Text Label 2900 5050 0    50   ~ 0
SCL
Text Notes 900  2350 0    50   ~ 0
TMP117: 3.22CAD/unit @ 500 units\n\nPullups can be a heat source, so maintain some distance.\nDo not solder thermal pad as it may introduce measurement error.\nRefer to Application Report SNOA986A to optimize surface measurement\n\nADD0 --- Address\nGND --- 0x48\nVDD --- 0x49\nSDA --- 0x4A\nSCL --- 0x4B
$Comp
L MCU_ST_STM32L4:STM32L432KBUx U6
U 1 1 61E3CE84
P 6400 3650
F 0 "U6" H 6400 4731 50  0000 C CNN
F 1 "STM32L432KBUx" H 6400 4640 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm" H 6000 2750 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00257205.pdf" H 6400 3650 50  0001 C CNN
	1    6400 3650
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR021
U 1 1 61E3E577
P 5950 2750
F 0 "#PWR021" H 5950 2600 50  0001 C CNN
F 1 "VDD" H 5967 2923 50  0000 C CNN
F 2 "" H 5950 2750 50  0001 C CNN
F 3 "" H 5950 2750 50  0001 C CNN
	1    5950 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 2750 6300 2750
Connection ~ 6300 2750
Wire Wire Line
	6300 2750 6400 2750
$Comp
L power:VDDA #PWR028
U 1 1 61E404A3
P 6850 2750
F 0 "#PWR028" H 6850 2600 50  0001 C CNN
F 1 "VDDA" H 6867 2923 50  0000 C CNN
F 2 "" H 6850 2750 50  0001 C CNN
F 3 "" H 6850 2750 50  0001 C CNN
	1    6850 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2750 6850 2750
Text Label 5900 2950 2    50   ~ 0
NRST
Text Label 5900 3350 2    50   ~ 0
BOOT0
Text Label 5900 3550 2    50   ~ 0
OSC_IN
Text Label 5900 3650 2    50   ~ 0
OSC_OUT
$Comp
L power:GND #PWR026
U 1 1 61E43522
P 6400 4650
F 0 "#PWR026" H 6400 4400 50  0001 C CNN
F 1 "GND" H 6405 4477 50  0000 C CNN
F 2 "" H 6400 4650 50  0001 C CNN
F 3 "" H 6400 4650 50  0001 C CNN
	1    6400 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 4650 6400 4650
Connection ~ 6400 4650
Wire Wire Line
	6400 4650 6500 4650
$Comp
L Device:C C5
U 1 1 61E45D82
P 4900 2100
F 0 "C5" H 5015 2146 50  0000 L CNN
F 1 "10uF" H 5015 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4938 1950 50  0001 C CNN
F 3 "~" H 4900 2100 50  0001 C CNN
	1    4900 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 61E46B62
P 5300 2100
F 0 "C6" H 5415 2146 50  0000 L CNN
F 1 "100nF" H 5415 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5338 1950 50  0001 C CNN
F 3 "~" H 5300 2100 50  0001 C CNN
	1    5300 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 61E4718F
P 5750 2100
F 0 "C7" H 5865 2146 50  0000 L CNN
F 1 "100nF" H 5865 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5788 1950 50  0001 C CNN
F 3 "~" H 5750 2100 50  0001 C CNN
	1    5750 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 61E47BD8
P 6700 2100
F 0 "C11" H 6815 2146 50  0000 L CNN
F 1 "100nF" H 6815 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6738 1950 50  0001 C CNN
F 3 "~" H 6700 2100 50  0001 C CNN
	1    6700 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 61E47EFD
P 6350 2100
F 0 "C9" H 6465 2146 50  0000 L CNN
F 1 "1uF" H 6465 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6388 1950 50  0001 C CNN
F 3 "~" H 6350 2100 50  0001 C CNN
	1    6350 2100
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR017
U 1 1 61E4E3F5
P 4900 1950
F 0 "#PWR017" H 4900 1800 50  0001 C CNN
F 1 "VDD" H 4917 2123 50  0000 C CNN
F 2 "" H 4900 1950 50  0001 C CNN
F 3 "" H 4900 1950 50  0001 C CNN
	1    4900 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 1950 5300 1950
Connection ~ 4900 1950
Connection ~ 5300 1950
Wire Wire Line
	5300 1950 5750 1950
$Comp
L power:GND #PWR018
U 1 1 61E4FE6C
P 4900 2250
F 0 "#PWR018" H 4900 2000 50  0001 C CNN
F 1 "GND" H 4905 2077 50  0000 C CNN
F 2 "" H 4900 2250 50  0001 C CNN
F 3 "" H 4900 2250 50  0001 C CNN
	1    4900 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 2250 5300 2250
Connection ~ 4900 2250
Connection ~ 5300 2250
Wire Wire Line
	5300 2250 5750 2250
$Comp
L power:VDDA #PWR027
U 1 1 61E557E4
P 6700 1950
F 0 "#PWR027" H 6700 1800 50  0001 C CNN
F 1 "VDDA" H 6717 2123 50  0000 C CNN
F 2 "" H 6700 1950 50  0001 C CNN
F 3 "" H 6700 1950 50  0001 C CNN
	1    6700 1950
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR022
U 1 1 61E56049
P 6350 1950
F 0 "#PWR022" H 6350 1800 50  0001 C CNN
F 1 "VDD" H 6367 2123 50  0000 C CNN
F 2 "" H 6350 1950 50  0001 C CNN
F 3 "" H 6350 1950 50  0001 C CNN
	1    6350 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 1950 6700 1950
Connection ~ 6350 1950
Connection ~ 6700 1950
$Comp
L power:GND #PWR023
U 1 1 61E56932
P 6350 2250
F 0 "#PWR023" H 6350 2000 50  0001 C CNN
F 1 "GND" H 6355 2077 50  0000 C CNN
F 2 "" H 6350 2250 50  0001 C CNN
F 3 "" H 6350 2250 50  0001 C CNN
	1    6350 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 2250 6700 2250
Connection ~ 6350 2250
$Comp
L Device:R R7
U 1 1 61E5B9FC
P 7650 2000
F 0 "R7" H 7800 1950 50  0000 C CNN
F 1 "10k" H 7800 2050 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" V 7580 2000 50  0001 C CNN
F 3 "~" H 7650 2000 50  0001 C CNN
	1    7650 2000
	0    1    1    0   
$EndComp
Text Label 7800 2000 0    50   ~ 0
BOOT0
$Comp
L power:GND #PWR029
U 1 1 61E5D4A6
P 7500 2000
F 0 "#PWR029" H 7500 1750 50  0001 C CNN
F 1 "GND" V 7505 1872 50  0000 R CNN
F 2 "" H 7500 2000 50  0001 C CNN
F 3 "" H 7500 2000 50  0001 C CNN
	1    7500 2000
	0    1    1    0   
$EndComp
Text Label 7800 2350 0    50   ~ 0
NRST
$Comp
L Device:C C13
U 1 1 61E61D59
P 7650 2350
F 0 "C13" V 7400 2400 50  0000 C CNN
F 1 "100nF" V 7500 2350 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7688 2200 50  0001 C CNN
F 3 "~" H 7650 2350 50  0001 C CNN
	1    7650 2350
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR030
U 1 1 61E63A29
P 7500 2350
F 0 "#PWR030" H 7500 2100 50  0001 C CNN
F 1 "GND" V 7505 2222 50  0000 R CNN
F 2 "" H 7500 2350 50  0001 C CNN
F 3 "" H 7500 2350 50  0001 C CNN
	1    7500 2350
	0    1    1    0   
$EndComp
Text Label 6900 3850 0    50   ~ 0
SCL
Text Label 6900 3950 0    50   ~ 0
SDA
Text Label 5900 4350 2    50   ~ 0
VCP_TX
Text Label 5900 4450 2    50   ~ 0
VCP_RX
Text Label 6900 4350 0    50   ~ 0
SWCLK
Text Label 6900 4250 0    50   ~ 0
SWDIO
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J1
U 1 1 61E7878A
P 5000 3100
F 0 "J1" H 5050 3417 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 5050 3326 50  0000 C CNN
F 2 "Connector:Tag-Connect_TC2030-IDC-NL_2x03_P1.27mm_Vertical" H 5000 3100 50  0001 C CNN
F 3 "~" H 5000 3100 50  0001 C CNN
	1    5000 3100
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR015
U 1 1 61E79537
P 4800 3000
F 0 "#PWR015" H 4800 2850 50  0001 C CNN
F 1 "VDD" V 4818 3127 50  0000 L CNN
F 2 "" H 4800 3000 50  0001 C CNN
F 3 "" H 4800 3000 50  0001 C CNN
	1    4800 3000
	0    -1   -1   0   
$EndComp
Text Label 4800 3100 2    50   ~ 0
NRST
$Comp
L power:GND #PWR016
U 1 1 61E7D7DA
P 4800 3200
F 0 "#PWR016" H 4800 2950 50  0001 C CNN
F 1 "GND" V 4805 3072 50  0000 R CNN
F 2 "" H 4800 3200 50  0001 C CNN
F 3 "" H 4800 3200 50  0001 C CNN
	1    4800 3200
	0    1    1    0   
$EndComp
Text Label 5300 3000 0    50   ~ 0
SWDIO
Text Label 5300 3100 0    50   ~ 0
SWCLK
NoConn ~ 5300 3200
$Comp
L Device:Crystal Y1
U 1 1 61E861DA
P 7600 3000
F 0 "Y1" H 7600 3268 50  0000 C CNN
F 1 "Crystal" H 7600 3177 50  0000 C CNN
F 2 "flowsensor:ECS-.327-12.5-12R-C-TR" H 7600 3000 50  0001 C CNN
F 3 "~" H 7600 3000 50  0001 C CNN
	1    7600 3000
	1    0    0    -1  
$EndComp
Text Label 7450 3000 2    50   ~ 0
OSC_IN
Text Label 7750 3000 0    50   ~ 0
OSC_OUT
$Comp
L Device:C C12
U 1 1 61E8E3D3
P 7450 3150
F 0 "C12" H 7700 3100 50  0000 R CNN
F 1 "13pF" H 7750 3200 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7488 3000 50  0001 C CNN
F 3 "~" H 7450 3150 50  0001 C CNN
	1    7450 3150
	-1   0    0    1   
$EndComp
$Comp
L Device:C C14
U 1 1 61E95D88
P 7750 3150
F 0 "C14" H 7600 3100 50  0000 R CNN
F 1 "13pF" H 7650 3200 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7788 3000 50  0001 C CNN
F 3 "~" H 7750 3150 50  0001 C CNN
	1    7750 3150
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR033
U 1 1 61E99C0D
P 7600 3300
F 0 "#PWR033" H 7600 3050 50  0001 C CNN
F 1 "GND" H 7605 3127 50  0000 C CNN
F 2 "" H 7600 3300 50  0001 C CNN
F 3 "" H 7600 3300 50  0001 C CNN
	1    7600 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3300 7600 3300
Connection ~ 7600 3300
Wire Wire Line
	7600 3300 7750 3300
Text Label 6900 3150 0    50   ~ 0
TX
Text Label 6900 3250 0    50   ~ 0
RX
Text Label 9600 3150 0    50   ~ 0
SWDIO
Text Label 9600 3350 0    50   ~ 0
SWCLK
Text Label 9600 3050 0    50   ~ 0
NRST
Text Label 9600 2100 0    50   ~ 0
TX
Text Label 9600 2000 0    50   ~ 0
RX
Text Label 9600 2950 0    50   ~ 0
VCP_TX
Text Label 9600 2850 0    50   ~ 0
VCP_RX
$Comp
L power:GND #PWR035
U 1 1 61EE067B
P 9600 2300
F 0 "#PWR035" H 9600 2050 50  0001 C CNN
F 1 "GND" V 9605 2172 50  0000 R CNN
F 2 "" H 9600 2300 50  0001 C CNN
F 3 "" H 9600 2300 50  0001 C CNN
	1    9600 2300
	0    -1   -1   0   
$EndComp
$Comp
L power:VDD #PWR034
U 1 1 61EE09AA
P 9600 2200
F 0 "#PWR034" H 9600 2050 50  0001 C CNN
F 1 "VDD" V 9617 2328 50  0000 L CNN
F 2 "" H 9600 2200 50  0001 C CNN
F 3 "" H 9600 2200 50  0001 C CNN
	1    9600 2200
	0    1    1    0   
$EndComp
$Comp
L Device:D_TVS D1
U 1 1 61EE97FC
P 7500 4150
F 0 "D1" V 7454 4229 50  0000 L CNN
F 1 "D_TVS" V 7545 4229 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 7500 4150 50  0001 C CNN
F 3 "~" H 7500 4150 50  0001 C CNN
	1    7500 4150
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR031
U 1 1 61EECC6A
P 7500 4000
F 0 "#PWR031" H 7500 3850 50  0001 C CNN
F 1 "VDD" H 7517 4173 50  0000 C CNN
F 2 "" H 7500 4000 50  0001 C CNN
F 3 "" H 7500 4000 50  0001 C CNN
	1    7500 4000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR032
U 1 1 61EEE732
P 7500 4300
F 0 "#PWR032" H 7500 4050 50  0001 C CNN
F 1 "GND" H 7505 4127 50  0000 C CNN
F 2 "" H 7500 4300 50  0001 C CNN
F 3 "" H 7500 4300 50  0001 C CNN
	1    7500 4300
	1    0    0    -1  
$EndComp
$Comp
L flowsensor:TC78H651AFNG,EL_Dual_H-Bridge U5
U 1 1 61F10101
P 9100 4300
F 0 "U5" H 9100 4875 50  0000 C CNN
F 1 "TC78H651AFNG,EL_Dual_H-Bridge" H 9100 4784 50  0000 C CNN
F 2 "flowsensor:TC78H651AFNG,EL_Dual_H-Bridge" H 9150 4850 50  0001 C CNN
F 3 "" H 9150 4850 50  0001 C CNN
	1    9100 4300
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR019
U 1 1 61F27F9F
P 8750 3950
F 0 "#PWR019" H 8750 3800 50  0001 C CNN
F 1 "VDD" V 8768 4077 50  0000 L CNN
F 2 "" H 8750 3950 50  0001 C CNN
F 3 "" H 8750 3950 50  0001 C CNN
	1    8750 3950
	0    -1   -1   0   
$EndComp
Text Label 6900 3350 0    50   ~ 0
PA4
Text Label 6900 3450 0    50   ~ 0
PA5
Text Label 6900 3550 0    50   ~ 0
PA6
Text Label 6900 3650 0    50   ~ 0
PA7
Text Label 8750 4550 2    50   ~ 0
PA4
Text Label 8750 4450 2    50   ~ 0
PA5
Text Label 8750 4250 2    50   ~ 0
PA6
Text Label 8750 4150 2    50   ~ 0
PA7
Text Label 9450 4050 0    50   ~ 0
OUT4
Text Label 9450 4150 0    50   ~ 0
OUT3
Text Label 9450 4450 0    50   ~ 0
OUT2
Text Label 9450 4350 0    50   ~ 0
OUT1
$Comp
L power:GND #PWR020
U 1 1 61F544AE
P 9450 4650
F 0 "#PWR020" H 9450 4400 50  0001 C CNN
F 1 "GND" H 9455 4477 50  0000 C CNN
F 2 "" H 9450 4650 50  0001 C CNN
F 3 "" H 9450 4650 50  0001 C CNN
	1    9450 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 61F558B5
P 9850 4550
F 0 "C8" H 9965 4596 50  0000 L CNN
F 1 "1uF" H 9965 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9888 4400 50  0001 C CNN
F 3 "~" H 9850 4550 50  0001 C CNN
	1    9850 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 61F55BC5
P 10250 4550
F 0 "C10" H 10365 4596 50  0000 L CNN
F 1 "100nF" H 10365 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10288 4400 50  0001 C CNN
F 3 "~" H 10250 4550 50  0001 C CNN
	1    10250 4550
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR024
U 1 1 61F6402B
P 10050 4400
F 0 "#PWR024" H 10050 4250 50  0001 C CNN
F 1 "VDD" H 10067 4573 50  0000 C CNN
F 2 "" H 10050 4400 50  0001 C CNN
F 3 "" H 10050 4400 50  0001 C CNN
	1    10050 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 4400 10050 4400
Connection ~ 10050 4400
Wire Wire Line
	10050 4400 10250 4400
$Comp
L power:GND #PWR025
U 1 1 61F65B40
P 10050 4700
F 0 "#PWR025" H 10050 4450 50  0001 C CNN
F 1 "GND" H 10055 4527 50  0000 C CNN
F 2 "" H 10050 4700 50  0001 C CNN
F 3 "" H 10050 4700 50  0001 C CNN
	1    10050 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 4700 10050 4700
Connection ~ 10050 4700
Wire Wire Line
	10050 4700 10250 4700
$Comp
L power:GND #PWR036
U 1 1 61F71225
P 9850 3250
F 0 "#PWR036" H 9850 3000 50  0001 C CNN
F 1 "GND" V 9855 3122 50  0000 R CNN
F 2 "" H 9850 3250 50  0001 C CNN
F 3 "" H 9850 3250 50  0001 C CNN
	1    9850 3250
	0    -1   -1   0   
$EndComp
$Comp
L power:VDD #PWR037
U 1 1 61F79AD8
P 9850 3450
F 0 "#PWR037" H 9850 3300 50  0001 C CNN
F 1 "VDD" V 9867 3578 50  0000 L CNN
F 2 "" H 9850 3450 50  0001 C CNN
F 3 "" H 9850 3450 50  0001 C CNN
	1    9850 3450
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x07 J4
U 1 1 61F85215
P 9400 3150
F 0 "J4" H 9318 2625 50  0000 C CNN
F 1 "Conn_01x07" H 9318 2716 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical" H 9400 3150 50  0001 C CNN
F 3 "~" H 9400 3150 50  0001 C CNN
	1    9400 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	9850 3450 9600 3450
Wire Wire Line
	9850 3250 9600 3250
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 61FA0579
P 9400 2200
F 0 "J3" H 9318 1775 50  0000 C CNN
F 1 "Conn_01x04" H 9318 1866 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 9400 2200 50  0001 C CNN
F 3 "~" H 9400 2200 50  0001 C CNN
	1    9400 2200
	-1   0    0    1   
$EndComp
$Comp
L Device:C C1
U 1 1 61FC9BB6
P 3300 2700
F 0 "C1" H 3185 2654 50  0000 R CNN
F 1 "100nF" H 3185 2745 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3338 2550 50  0001 C CNN
F 3 "~" H 3300 2700 50  0001 C CNN
	1    3300 2700
	-1   0    0    1   
$EndComp
Wire Wire Line
	3300 2550 2900 2550
Connection ~ 3300 2550
$Comp
L power:GND #PWR08
U 1 1 61FD5A94
P 3300 2850
F 0 "#PWR08" H 3300 2600 50  0001 C CNN
F 1 "GND" H 3305 2677 50  0000 C CNN
F 2 "" H 3300 2850 50  0001 C CNN
F 3 "" H 3300 2850 50  0001 C CNN
	1    3300 2850
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR09
U 1 1 61FD9696
P 3300 3350
F 0 "#PWR09" H 3300 3200 50  0001 C CNN
F 1 "VDD" H 3317 3523 50  0000 C CNN
F 2 "" H 3300 3350 50  0001 C CNN
F 3 "" H 3300 3350 50  0001 C CNN
	1    3300 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 61FD969C
P 3300 3500
F 0 "C2" H 3185 3454 50  0000 R CNN
F 1 "100nF" H 3185 3545 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3338 3350 50  0001 C CNN
F 3 "~" H 3300 3500 50  0001 C CNN
	1    3300 3500
	-1   0    0    1   
$EndComp
Wire Wire Line
	3300 3350 2900 3350
Connection ~ 3300 3350
$Comp
L power:GND #PWR010
U 1 1 61FD96A4
P 3300 3650
F 0 "#PWR010" H 3300 3400 50  0001 C CNN
F 1 "GND" H 3305 3477 50  0000 C CNN
F 2 "" H 3300 3650 50  0001 C CNN
F 3 "" H 3300 3650 50  0001 C CNN
	1    3300 3650
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR011
U 1 1 61FE8BCA
P 3300 4150
F 0 "#PWR011" H 3300 4000 50  0001 C CNN
F 1 "VDD" H 3317 4323 50  0000 C CNN
F 2 "" H 3300 4150 50  0001 C CNN
F 3 "" H 3300 4150 50  0001 C CNN
	1    3300 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 61FE8BD0
P 3300 4300
F 0 "C3" H 3185 4254 50  0000 R CNN
F 1 "100nF" H 3185 4345 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3338 4150 50  0001 C CNN
F 3 "~" H 3300 4300 50  0001 C CNN
	1    3300 4300
	-1   0    0    1   
$EndComp
Wire Wire Line
	3300 4150 2900 4150
Connection ~ 3300 4150
$Comp
L power:GND #PWR012
U 1 1 61FE8BD8
P 3300 4450
F 0 "#PWR012" H 3300 4200 50  0001 C CNN
F 1 "GND" H 3305 4277 50  0000 C CNN
F 2 "" H 3300 4450 50  0001 C CNN
F 3 "" H 3300 4450 50  0001 C CNN
	1    3300 4450
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR013
U 1 1 61FEFDE0
P 3300 4950
F 0 "#PWR013" H 3300 4800 50  0001 C CNN
F 1 "VDD" H 3317 5123 50  0000 C CNN
F 2 "" H 3300 4950 50  0001 C CNN
F 3 "" H 3300 4950 50  0001 C CNN
	1    3300 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 61FEFDE6
P 3300 5100
F 0 "C4" H 3185 5054 50  0000 R CNN
F 1 "100nF" H 3185 5145 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3338 4950 50  0001 C CNN
F 3 "~" H 3300 5100 50  0001 C CNN
	1    3300 5100
	-1   0    0    1   
$EndComp
Wire Wire Line
	3300 4950 2900 4950
Connection ~ 3300 4950
$Comp
L power:GND #PWR014
U 1 1 61FEFDEE
P 3300 5250
F 0 "#PWR014" H 3300 5000 50  0001 C CNN
F 1 "GND" H 3305 5077 50  0000 C CNN
F 2 "" H 3300 5250 50  0001 C CNN
F 3 "" H 3300 5250 50  0001 C CNN
	1    3300 5250
	1    0    0    -1  
$EndComp
Wire Notes Line
	800  5550 3900 5550
Wire Notes Line
	3900 5550 3900 1350
Wire Notes Line
	3900 1350 800  1350
Wire Notes Line
	800  1350 800  5550
Text Notes 4800 1650 0    50   ~ 0
STM32L432KB MCU
Wire Notes Line
	4450 5000 8350 5000
Wire Notes Line
	8350 5000 8350 1450
Wire Notes Line
	8350 1450 4450 1450
Wire Notes Line
	4450 1450 4450 5000
Text Notes 9250 2550 0    50   ~ 0
Prog & Debug
Wire Notes Line
	9200 2450 9200 3550
Wire Notes Line
	9200 3550 10150 3550
Wire Notes Line
	10150 3550 10150 2450
Wire Notes Line
	10150 2450 9200 2450
Text Notes 9250 1700 0    50   ~ 0
Data Interface
Wire Notes Line
	9200 1600 9200 2400
Wire Notes Line
	9200 2400 10100 2400
Wire Notes Line
	10100 2400 10100 1600
Wire Notes Line
	10100 1600 9200 1600
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 620ED13F
P 10200 3950
F 0 "J2" H 10280 3942 50  0000 L CNN
F 1 "Conn_01x02" H 10280 3851 50  0000 L CNN
F 2 "Resistor_THT:R_Box_L8.4mm_W2.5mm_P5.08mm" H 10200 3950 50  0001 C CNN
F 3 "~" H 10200 3950 50  0001 C CNN
	1    10200 3950
	1    0    0    -1  
$EndComp
Text Label 10000 3950 2    50   ~ 0
OUT4
Text Label 10000 4050 2    50   ~ 0
OUT3
$Comp
L Mechanical:MountingHole H1
U 1 1 62166020
P 8500 2750
F 0 "H1" H 8600 2796 50  0000 L CNN
F 1 "MountingHole" H 8600 2705 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 8500 2750 50  0001 C CNN
F 3 "~" H 8500 2750 50  0001 C CNN
	1    8500 2750
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 6216A891
P 8500 2950
F 0 "H2" H 8600 2996 50  0000 L CNN
F 1 "MountingHole" H 8600 2905 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 8500 2950 50  0001 C CNN
F 3 "~" H 8500 2950 50  0001 C CNN
	1    8500 2950
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 6216AA63
P 8500 3150
F 0 "H3" H 8600 3196 50  0000 L CNN
F 1 "MountingHole" H 8600 3105 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 8500 3150 50  0001 C CNN
F 3 "~" H 8500 3150 50  0001 C CNN
	1    8500 3150
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 6216AD71
P 8500 3350
F 0 "H4" H 8600 3396 50  0000 L CNN
F 1 "MountingHole" H 8600 3305 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 8500 3350 50  0001 C CNN
F 3 "~" H 8500 3350 50  0001 C CNN
	1    8500 3350
	1    0    0    -1  
$EndComp
NoConn ~ 2900 5150
NoConn ~ 2900 4350
NoConn ~ 2900 3550
NoConn ~ 2900 2750
Wire Notes Line
	8450 3600 8450 4950
Wire Notes Line
	8450 4950 10750 4950
Wire Notes Line
	10750 4950 10750 3600
Wire Notes Line
	10750 3600 8450 3600
Text Notes 8500 3700 0    50   ~ 0
Heater
$EndSCHEMATC
