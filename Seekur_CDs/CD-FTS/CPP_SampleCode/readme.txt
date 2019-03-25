FTLDemo
Version 1.2, June 2013

C++ demo program to show how to get force torque measurements from the FTL sensor.


Requirements:
* FTL Sensor with USB2CAN bridge
* C++ compiler
* boost libaries
The software has been developed on Ubuntu Linux with gcc compiler.


Functionalities:
* Connects via a virtual com port with the FTL sensor
* Establishes a communication loop to request and receive measurement data
* Multiply the measurements with a generic calibration matrix to receive XYZRXRYRZ values
* Print the force/torque values on the screen


How to run?
* Compile the sources using the /Debug/makefile or another development environment
* run ./FTLDemo without any arguments
* the first 15 lines will be used to gather a bias value, then the xyz values are shown

Trouble shooting:
* USB2CAN controller not found? Check if it is on /dev/ttyUSB0, when not change in RS232CAN.cpp
* No access to USB2CAN controller? Set user rights with chmod or run as admin (sudo ./FTLDemo)


This program cannot serve a measurement tool, it is only intended to provide guidance when connecting to the sensor!
The generic calibration matrix has to be replaced by the sensor specific matrix provided with the sensor.




Version history:
1.2 06/2013 Matrix multiplication and temperature included
1.0 04/2013 Start version

