EESchema Schematic File Version 2  date Sunday, March 18, 2012 02:00:55 AM
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
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
LIBS:usbkey-cache
EELAYER 25  0
EELAYER END
$Descr User 11000 8500
encoding utf-8
Sheet 1 1
Title ""
Date "10 mar 2012"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	3450 4900 3450 5050
Wire Wire Line
	3450 5050 3250 5050
Wire Wire Line
	3150 3350 2800 3350
Wire Wire Line
	2350 3350 2400 3350
Wire Wire Line
	2300 2450 2300 2350
Wire Wire Line
	2700 3200 1550 3200
Wire Wire Line
	3900 3600 3900 3500
Wire Wire Line
	3900 3500 3950 3500
Wire Wire Line
	3950 3200 3700 3200
Wire Wire Line
	3700 3200 3700 3100
Wire Wire Line
	3700 3100 3200 3100
Connection ~ 6750 3000
Wire Wire Line
	6750 3050 6750 2900
Wire Wire Line
	6750 3000 6650 3000
Wire Wire Line
	1550 3150 1550 3100
Wire Wire Line
	1550 3450 1700 3450
Wire Wire Line
	1550 3200 1550 3250
Wire Wire Line
	6750 3500 6650 3500
Wire Wire Line
	6750 3450 6750 3650
Connection ~ 6750 3500
Wire Wire Line
	3200 3200 3350 3200
Wire Wire Line
	3350 3200 3350 3000
Wire Wire Line
	3350 3000 3950 3000
Wire Wire Line
	3950 3100 3800 3100
Wire Wire Line
	3800 3100 3800 2850
Wire Wire Line
	3450 4000 3450 3300
Wire Wire Line
	3450 3300 3950 3300
Wire Wire Line
	3450 4300 3450 4400
Wire Wire Line
	1700 3450 1700 3750
Wire Wire Line
	1550 3100 2700 3100
Wire Wire Line
	2300 3200 2300 2950
Connection ~ 2300 3200
Connection ~ 1900 3350
Wire Wire Line
	1950 3350 1550 3350
$Comp
L LED3MM LED1
U 1 1 4F5983DF
P 3450 4200
F 0 "LED1" V 3350 4150 50  0000 L BNN
F 1 "LED3MM" V 3550 4050 50  0000 L BNN
F 2 "LED-3MM" H 3450 4350 50  0001 C CNN
	1    3450 4200
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR01
U 1 1 4F59A7D4
P 3250 5050
F 0 "#PWR01" H 3250 5140 20  0001 C CNN
F 1 "+5V" H 3250 5140 30  0000 C CNN
	1    3250 5050
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR02
U 1 1 4F59A798
P 3150 3350
F 0 "#PWR02" H 3150 3310 30  0001 C CNN
F 1 "+3.3V" H 3150 3460 30  0000 C CNN
	1    3150 3350
	1    0    0    -1  
$EndComp
$Comp
L DIODE D2
U 1 1 4F59A711
P 2600 3350
F 0 "D2" H 2600 3450 40  0000 C CNN
F 1 "1N4148" H 2600 3250 40  0000 C CNN
	1    2600 3350
	1    0    0    -1  
$EndComp
$Comp
L DIODE D1
U 1 1 4F59A709
P 2150 3350
F 0 "D1" H 2150 3450 40  0000 C CNN
F 1 "1N4148" H 2150 3250 40  0000 C CNN
	1    2150 3350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR03
U 1 1 4F59A701
P 1900 3350
F 0 "#PWR03" H 1900 3440 20  0001 C CNN
F 1 "+5V" H 1900 3440 30  0000 C CNN
	1    1900 3350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 4F599A08
P 2300 2350
F 0 "#PWR04" H 2300 2310 30  0001 C CNN
F 1 "+3.3V" H 2300 2460 30  0000 C CNN
	1    2300 2350
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 4F5999F3
P 2300 2700
F 0 "R1" V 2380 2700 50  0000 C CNN
F 1 "1.5kR" V 2300 2700 50  0000 C CNN
	1    2300 2700
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR05
U 1 1 4F5989D0
P 1700 3750
F 0 "#PWR05" H 1700 3750 30  0001 C CNN
F 1 "GND" H 1700 3680 30  0001 C CNN
	1    1700 3750
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 4F5987F2
P 3450 4650
F 0 "R4" V 3530 4650 50  0000 C CNN
F 1 "R" V 3450 4650 50  0000 C CNN
	1    3450 4650
	-1   0    0    1   
$EndComp
$Comp
L CONN_1 P1
U 1 1 4F5976B8
P 3800 2700
F 0 "P1" H 3880 2700 40  0000 L CNN
F 1 "CONN_1" H 3800 2755 30  0001 C CNN
	1    3800 2700
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P2
U 1 1 4F5976AA
P 3900 3750
F 0 "P2" H 3980 3750 40  0000 L CNN
F 1 "CONN_1" H 3900 3805 30  0001 C CNN
	1    3900 3750
	0    1    1    0   
$EndComp
$Comp
L C C2
U 1 1 4F596C53
P 6750 3250
F 0 "C2" H 6800 3350 50  0000 L CNN
F 1 "0.1uF" H 6800 3150 50  0000 L CNN
	1    6750 3250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 4F596C4A
P 6750 3650
F 0 "#PWR06" H 6750 3650 30  0001 C CNN
F 1 "GND" H 6750 3580 30  0001 C CNN
	1    6750 3650
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR07
U 1 1 4F596C3F
P 6750 2900
F 0 "#PWR07" H 6750 2860 30  0001 C CNN
F 1 "+3.3V" H 6750 3010 30  0000 C CNN
	1    6750 2900
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 4F596681
P 2950 3200
F 0 "R3" V 3030 3200 50  0000 C CNN
F 1 "68R" V 2950 3200 50  0000 C CNN
	1    2950 3200
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 4F596678
P 2950 3100
F 0 "R2" V 3030 3100 50  0000 C CNN
F 1 "68R" V 2950 3100 50  0000 C CNN
	1    2950 3100
	0    -1   -1   0   
$EndComp
$Comp
L USB-A-S J1
U 1 1 4F59650C
P 1450 3450
F 0 "J1" H 1550 3850 50  0000 C CNN
F 1 "USB-A-S" H 1550 3350 50  0000 C CNN
F 2 "usb_pcb_trace" H 1550 3250 50  0001 C CNN
	1    1450 3450
	-1   0    0    -1  
$EndComp
$Comp
L ATTINY85-P IC1
U 1 1 4F59641F
P 4150 2900
F 0 "IC1" H 4250 2950 60  0000 C CNN
F 1 "ATTINY85-P" H 6150 2150 60  0000 C CNN
F 2 "DIP8" H 4250 2150 60  0001 C CNN
	1    4150 2900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
