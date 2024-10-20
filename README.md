# Overview
This is a heat pump controler. It works for pumps that can be controled by 0-10V signal.
I run it on Rotenso Mirai X and AHU RCU-AHUBOX-1C. The project is run on both microcontrollers Arduino Nano (ATmega 328p)
that is required and Wemos D1 mini (esp8266) that is optional. This is because Control Module needs to be reliable and connot be disturbed
by external conditions and Communication Module is optional. There is only one exception from this rule with energy meter. It is connected
directly to Control Module because of available pins.

# Features
- 4-point linear approximated heat curve for -5℃, 0℃, 5℃ and 10℃.
- Wired ethernet connection support
- Integrated with Domoticz by MQTT
- MQTT based console line interface
- Autonomous heat pump control
- Water flow stop protection
- Statistics support
- PID inside
- Easy to run

# Modules
The project is run using following devices and modules.

## Control module
- AHU RCU-AHUBOX-1C
- Arduino Nano 328p
- 3x DS18B20 temperature sensors
- PWM to 0-10V converter
- Pulse flow meter
- PZEM-004 (optional)

## Communication module
- Wemos D1 mini (esp8266)
- ENC28J60
- Voltage level converter

# Module details

## AHU RCU-AHUBOX-1C
This is official Rotenso unit that replaced indoor unit and can be controlled by following signals:
Input:
  0-10V that represents the power but it is not true. Read about it later.
  heat request
  cool request (I don't use it)
Output:
  defrost relay
  alarm relay (I don't use it. Never seen it is active)

## Arduino Nano 328p
Chineese clone of Arduino Nano. You can also use bigger Arduino modules.

## DS18B20
I use 3 thermometers Maxim Dallas DS18B20. They measure heat water temperature, water return temperature and outdoor temperature.
Make sure you're using genuine Maxim modules. They are much more accurate than chineese fakes. Outdoor temperature are used to
calculate heating temperature by 4-point, linear approximated heat curve. Modules can be easly bind to their function by 3 buttons.
Connect the DS18B20 module to the D5 pin and hold the Heat, Return or Outside button for 3 seconds. Now you can connect the module
to the main bus on D4. Readings will be bind to the appropriate functions. Heat and Outside temperatures are required for proper
operation. Return temperature is optional and used for statistics only.

## PWM to 0-10V converter
I use chineese PWM to 0-10V converter. The controller uses PWM on their output to control the heat power. It comverts to 0-10V signal
that is accepted by AHUBOX.

## Pulse flow meter
I use chineese bras flow meter. It's assembled with turbine and hall sensor that gives us pulses. Some peaple use them to calculate
flow. For the project purposes I used it for flow problems detection. If the pulse frequency drops the threshold it raises alarm
and switch off the heat pump. It's connected to interupt input of Control Module. The stable flow on a good level is very important
during defrost operation. It protects heat exchanger from freezing. Freezed heat exchanger may leak and the water can be sucked
by the compressor inside the refrigeration installation and damage the unit.

## PZEM-004T
It is used for electricity measurement for statistics purposes. It measures voltage, current, power and power factor. It will be used
for COP calculating in the feature. It is connected to Control Module using Software UART interface.

## Wemos D1 mini (esp8266)
It is used for optional communication with the control module. It allows you to set and read operation parameters. It is integrated
with Ethernet, MQTT and Domoticz. It also supports simple command line interface using 2 MQTT topics. It is connected to the Control
Module using I2C interface by voltage level converter. It's required because Arduino Nano is working in 5V logic and ESP8266 is
working in 3.3V logic.

## ENC28J60
It is a wired ethernet connection. Wemos D1 mini supports Wi-Fi connection and it can be use in the future to have network connection.
My goal is to have it reliable so I prefer wired network connection. The module is an interface between network and Communication
Module using SPI interface.

## Voltage level converter
Control Module and Communication Module have to be connected together but they are working on different logic voltage levels. Arduino
Nano works on 5V logic level, ESP8266 works on 3.3V logic level. It has to be connected by voltage converter. Direct connection may
damage Communication Module that uses ESP8266 module. The connection uses I2C interface.

# Additional information

## AHU RCU-AHUBOX-1C
Many people tried to understand what algorithm is used to control outdoor unit by AHUBOX. Based on information from the Internet
debugging and reverse engineering I have my own thoughts. it looks like it works as follow. AHUBOX has the 0-10V interface. It has 10
thresholds on following voltage levels:

| threshold | input voltage |
|-----------|---------------|
| 00        | 0.0V - 0.5V   |
| 01        | 0.5V - 1.5V   |
| 02        | 1.5V - 2.5V   |
| 03        | 2.5V - 3.5V   |
| 04        | 3.5V - 4.5V   |
| 05        | 4.5V - 5.5V   |
| 06        | 5.5V - 6.5V   |
| 07        | 6.5V - 7.5V   |
| 08        | 7.5V - 8.5V   |
| 09        | 8.5V - 9.5V   |
| 10        | 9.5V - 10.5V  |

Each voltage level cause changing the message that is sent by AHUBOX to the outdoor unit. The outdoor unit receives some temperatures
like indoor temperature, setpoint indoor temperature, heat exchange temperature. AHUBOX sends indoor temperature always as 23 deg C
and setpoint indoor temperature depends on the voltage level threshold.

00 switches off the heat pump\
01 sends the setpoint indoor temperature below 23℃\
02 sends the setpoint indoor remperature 23℃\
03 and higher sends the setpoint indoor temperature over 23℃

In the outdoor unit the controller uses it's own PID to control the power and engine speed of the compressor. The PID uses setpoint
indoor temperature (that is always 23℃) as Set Value and indoor temperature as Process Value. PV depends on the level of voltage
threshold. In short: 01 decreases the power, 02 keeps the power, 03 and above increases the power. The power is increased in the speed
dependent on the voltage threshold.
