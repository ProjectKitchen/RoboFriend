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
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L VCC #PWR01
U 1 1 5AD4929C
P 2800 2500
F 0 "#PWR01" H 2800 2350 50  0001 C CNN
F 1 "VCC" H 2800 2650 50  0000 C CNN
F 2 "" H 2800 2500 50  0001 C CNN
F 3 "" H 2800 2500 50  0001 C CNN
	1    2800 2500
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5AD492DA
P 2800 2950
F 0 "R1" V 2880 2950 50  0000 C CNN
F 1 "R" V 2800 2950 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 2730 2950 50  0001 C CNN
F 3 "" H 2800 2950 50  0001 C CNN
	1    2800 2950
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 5AD49347
P 2800 3500
F 0 "D1" H 2800 3600 50  0000 C CNN
F 1 "LED" H 2800 3400 50  0000 C CNN
F 2 "LEDs:LED_0805" H 2800 3500 50  0001 C CNN
F 3 "" H 2800 3500 50  0001 C CNN
	1    2800 3500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2800 2500 2800 2800
Wire Wire Line
	2800 3100 2800 3350
$Comp
L GND #PWR02
U 1 1 5AD49443
P 2800 4050
F 0 "#PWR02" H 2800 3800 50  0001 C CNN
F 1 "GND" H 2800 3900 50  0000 C CNN
F 2 "" H 2800 4050 50  0001 C CNN
F 3 "" H 2800 4050 50  0001 C CNN
	1    2800 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 3650 2800 4050
$EndSCHEMATC
