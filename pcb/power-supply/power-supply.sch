EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:power-supply-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Power Supply"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Conn_01x20_Female J1
U 1 1 5B00326F
P 1400 2800
F 0 "J1" H 1400 3800 50  0000 C CNN
F 1 "Conn_01x20_Female" H 1400 1700 50  0000 C CNN
F 2 "" H 1400 2800 50  0001 C CNN
F 3 "" H 1400 2800 50  0001 C CNN
	1    1400 2800
	1    0    0    -1  
$EndComp
Text Label 950  1900 0    60   ~ 0
GND
Text Label 950  2000 0    60   ~ 0
B7
Text Label 950  2100 0    60   ~ 0
D0
Text Label 950  2200 0    60   ~ 0
D1
Text Label 950  2300 0    60   ~ 0
D2
Text Label 950  2400 0    60   ~ 0
D3
Text Label 950  2500 0    60   ~ 0
D4
Text Label 950  2600 0    60   ~ 0
D5
Text Label 950  2700 0    60   ~ 0
D6
Text Label 950  2800 0    60   ~ 0
D7
Text Label 950  2900 0    60   ~ 0
E0
Text Label 950  3000 0    60   ~ 0
E1
Text Label 950  3100 0    60   ~ 0
C0
Text Label 950  3200 0    60   ~ 0
C1
Text Label 950  3300 0    60   ~ 0
C2
Text Label 950  3400 0    60   ~ 0
C3
Text Label 950  3500 0    60   ~ 0
C4
Text Label 950  3600 0    60   ~ 0
C5
Text Label 950  3700 0    60   ~ 0
C6
Text Label 950  3800 0    60   ~ 0
c7
$Comp
L Conn_01x20_Female J2
U 1 1 5B003B4B
P 1750 2800
F 0 "J2" H 1750 3800 50  0000 C CNN
F 1 "Conn_01x20_Female" H 1750 1700 50  0000 C CNN
F 2 "" H 1750 2800 50  0001 C CNN
F 3 "" H 1750 2800 50  0001 C CNN
	1    1750 2800
	-1   0    0    -1  
$EndComp
Text Label 2100 1900 0    60   ~ 0
5V
Text Label 2100 2000 0    60   ~ 0
B6
Text Label 2100 2100 0    60   ~ 0
B5
Text Label 2100 2200 0    60   ~ 0
B4
Text Label 2100 2300 0    60   ~ 0
B3
Text Label 2100 2400 0    60   ~ 0
B2
Text Label 2100 2500 0    60   ~ 0
B1
Text Label 2100 3800 0    60   ~ 0
F7
Text Label 2100 3700 0    60   ~ 0
F6
Text Label 2100 3600 0    60   ~ 0
F5
Text Label 2100 3500 0    60   ~ 0
F4
Text Label 2100 3400 0    60   ~ 0
F3
Text Label 2100 3300 0    60   ~ 0
F2
Text Label 2100 3200 0    60   ~ 0
F1
Text Label 2100 3100 0    60   ~ 0
F0
Text Label 2100 3000 0    60   ~ 0
VREF
Text Label 2100 2700 0    60   ~ 0
E7
Text Label 2100 2800 0    60   ~ 0
E6
Text Label 2100 2600 0    60   ~ 0
B0
Text Label 2100 2900 0    60   ~ 0
GND
$Comp
L +5V #PWR1
U 1 1 5B003D2F
P 3800 750
F 0 "#PWR1" H 3800 600 50  0001 C CNN
F 1 "+5V" H 3800 890 50  0000 C CNN
F 2 "" H 3800 750 50  0001 C CNN
F 3 "" H 3800 750 50  0001 C CNN
	1    3800 750 
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5B003D4D
P 3800 1050
F 0 "R1" V 3880 1050 50  0000 C CNN
F 1 "150" V 3800 1050 50  0000 C CNN
F 2 "" V 3730 1050 50  0001 C CNN
F 3 "" H 3800 1050 50  0001 C CNN
	1    3800 1050
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 5B003DA4
P 3800 1500
F 0 "D1" H 3800 1600 50  0000 C CNN
F 1 "LED" H 3800 1400 50  0000 C CNN
F 2 "" H 3800 1500 50  0001 C CNN
F 3 "" H 3800 1500 50  0001 C CNN
	1    3800 1500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR2
U 1 1 5B003DFD
P 3800 1800
F 0 "#PWR2" H 3800 1550 50  0001 C CNN
F 1 "GND" H 3800 1650 50  0000 C CNN
F 2 "" H 3800 1800 50  0001 C CNN
F 3 "" H 3800 1800 50  0001 C CNN
	1    3800 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 750  3800 900 
Wire Wire Line
	3800 1200 3800 1350
Wire Wire Line
	3800 1650 3800 1800
Wire Wire Line
	1950 1900 2100 1900
Wire Wire Line
	1950 2000 2100 2000
Wire Wire Line
	1950 2100 2100 2100
Wire Wire Line
	1950 2200 2100 2200
Wire Wire Line
	1950 2300 2100 2300
Wire Wire Line
	2100 2400 1950 2400
Wire Wire Line
	1950 2500 2100 2500
Wire Wire Line
	1950 2600 2100 2600
Wire Wire Line
	1950 2700 2100 2700
Wire Wire Line
	1950 2800 2100 2800
Wire Wire Line
	1950 2900 2100 2900
Wire Wire Line
	1950 3000 2100 3000
Wire Wire Line
	1950 3100 2100 3100
Wire Wire Line
	1950 3200 2100 3200
Wire Wire Line
	1950 3300 2100 3300
Wire Wire Line
	1950 3400 2100 3400
Wire Wire Line
	1950 3500 2100 3500
Wire Wire Line
	1950 3600 2100 3600
Wire Wire Line
	1950 3700 2100 3700
Wire Wire Line
	1950 3800 2100 3800
Wire Wire Line
	950  1900 1200 1900
Wire Wire Line
	950  2000 1200 2000
Wire Wire Line
	1200 2100 950  2100
Wire Wire Line
	950  2200 1200 2200
Wire Wire Line
	1200 2300 950  2300
Wire Wire Line
	950  2400 1200 2400
Wire Wire Line
	950  2500 1200 2500
Wire Wire Line
	950  2600 1200 2600
Wire Wire Line
	950  2700 1200 2700
Wire Wire Line
	950  2800 1200 2800
Wire Wire Line
	950  2900 1200 2900
Wire Wire Line
	950  3000 1200 3000
Wire Wire Line
	950  3100 1200 3100
Wire Wire Line
	950  3200 1200 3200
Wire Wire Line
	950  3300 1200 3300
Wire Wire Line
	950  3400 1200 3400
Wire Wire Line
	950  3500 1200 3500
Wire Wire Line
	950  3600 1200 3600
Wire Wire Line
	950  3700 1200 3700
Wire Wire Line
	950  3800 1200 3800
Text Notes 4000 1550 0    60   ~ 0
current: 10 mA\nforward voltage: 2.1 V (green)
$Comp
L D D?
U 1 1 5B05C2AC
P 8500 1000
F 0 "D?" H 8500 1100 50  0000 C CNN
F 1 "D" H 8500 900 50  0000 C CNN
F 2 "" H 8500 1000 50  0001 C CNN
F 3 "" H 8500 1000 50  0001 C CNN
	1    8500 1000
	0    1    1    0   
$EndComp
$Comp
L D D?
U 1 1 5B05C312
P 8500 1500
F 0 "D?" H 8500 1600 50  0000 C CNN
F 1 "D" H 8500 1400 50  0000 C CNN
F 2 "" H 8500 1500 50  0001 C CNN
F 3 "" H 8500 1500 50  0001 C CNN
	1    8500 1500
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5B05C34C
P 8200 1250
F 0 "R?" V 8280 1250 50  0000 C CNN
F 1 "R" V 8200 1250 50  0000 C CNN
F 2 "" V 8130 1250 50  0001 C CNN
F 3 "" H 8200 1250 50  0001 C CNN
	1    8200 1250
	0    1    1    0   
$EndComp
Wire Wire Line
	8350 1250 8850 1250
Wire Wire Line
	8500 1350 8500 1150
Connection ~ 8500 1250
$Comp
L +5V #PWR?
U 1 1 5B05C556
P 8500 750
F 0 "#PWR?" H 8500 600 50  0001 C CNN
F 1 "+5V" H 8500 890 50  0000 C CNN
F 2 "" H 8500 750 50  0001 C CNN
F 3 "" H 8500 750 50  0001 C CNN
	1    8500 750 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B05C579
P 8500 1700
F 0 "#PWR?" H 8500 1450 50  0001 C CNN
F 1 "GND" H 8500 1550 50  0000 C CNN
F 2 "" H 8500 1700 50  0001 C CNN
F 3 "" H 8500 1700 50  0001 C CNN
	1    8500 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 750  8500 850 
Wire Wire Line
	8500 1650 8500 1700
$Comp
L Conn_01x02 J?
U 1 1 5B05C693
P 7850 1250
F 0 "J?" H 7850 1350 50  0000 C CNN
F 1 "Conn_01x02" H 7850 1050 50  0000 C CNN
F 2 "" H 7850 1250 50  0001 C CNN
F 3 "" H 7850 1250 50  0001 C CNN
	1    7850 1250
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B05C783
P 8150 1450
F 0 "#PWR?" H 8150 1200 50  0001 C CNN
F 1 "GND" H 8150 1300 50  0000 C CNN
F 2 "" H 8150 1450 50  0001 C CNN
F 3 "" H 8150 1450 50  0001 C CNN
	1    8150 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 1350 8150 1350
Wire Wire Line
	8150 1350 8150 1450
Text Label 8850 1250 0    60   ~ 0
F0
$EndSCHEMATC
