![image](images/header.PNG)

This fork contributes DC motor velocity control. This frees up the step/dir pins which are used instead as a second UART which has been implemented for constant velocity (RPM) setpoint input. It also frees up one of the motor drivers channels, could potentially use both in parallel to drive higher current. - Mark Lagana 10/1/23

## Bootloader:
You will need to make sure your Mechaduino has the latest Arduino Zero bootloader.  If you got your Mechaduino from Tropical Labs, it will already have this! Otherwise you will need an Atmel-ICE or similar SWD programmer...

## Firmware:

Compile the Mechaduino firmware in the Arduino IDE and upload to your Mechaduino.  (Mechaduino will appear as an Arduino Zero.)  

Original stepper firmware and older versions are available [here](https://github.com/jcchurch13/Mechaduino-Firmware/releases). 

## DC Motor Control:

Added control option from Serial1 UART channel

Serial1 uses the step/dir pins which were previously used for stepper control and are level shifted to 5v

Controller firmware available in commander directory. Uses arduino Esplora

Serial1 frame structure: (currently only velocity control)


## Velocity Commands (Serial1, rx & tx):

char command | velocity | terminator ('q')

velocity (RPM) is an unsigned int

d - clockwise (closed loop)

u - anti clockwise (closed loop)

x - clockwise (open loop)

z - anti clockwise (open loop)

s - stop

## Basic Commands (Serial0, USB):

Also Controlled via a SerialUSB terminal
Implemented SerialUSB commands are:

p  -  print [angle] , [encoder reading]

e  -  check encoder diagnositics

q  -  parameter query


x  -  position mode

v  -  velocity mode


y  -  enable control loop

n  -  disable control loop


r  -  enter new setpoint

k  -  edit controller gains -- note, these edits are stored in volatile memory and will be reset if power is cycled

m  -  print main menu


##License

All Mechaduino related materials are released under the
[Creative Commons Attribution Share-Alike 4.0 License](https://creativecommons.org/licenses/by-sa/4.0/)
