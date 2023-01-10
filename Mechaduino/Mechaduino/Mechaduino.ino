
/*
  -------------------------------------------------------------
  Mechaduino 0.1 & 0.2 Firmware  v0.1.5
  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  All Mechaduino related materials are released under the
  Creative Commons Attribution Share-Alike 4.0 License
  https://creativecommons.org/licenses/by-sa/4.0/

  Many thanks to all contributors!
  --------------------------------------------------------------
  
  This fork contributes DC motor velocity control - Mark Lagana 10/1/23
 
  * Added control option from Serial1 UART channel
  * Serial1 uses the step/dir pins which were previously used for stepper control and are level shifted to 5v
  * Controller code available in commander directory. Uses arduino Esplora

  Serial1 frame structure: (currently only velocity control)

  char command | velocity | terminator ('q')

  velocity (RPM) is an unsigned int (cast to string to transmit)

  commands:
  d - clockwise (closed loop)
  u - anti clockwise (closed loop)
  x - clockwise (open loop)
  z - anti clockwise (open loop)
  s - stop

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

*/

#include "Utils.h"
#include "Parameters.h"
#include "State.h" 
#include "analogFastWrite.h"

void setup()        // This code runs once at startup
{                         
  digitalWrite(ledPin,HIGH);        // turn LED on 
  setupPins();                      // configure pins
  setupTCInterrupts();              // configure controller interrupt
  SerialUSB.begin(9600);
  Serial1.begin(115200);          
  delay(3000);                      // This delay seems to make it easier to establish a connection when the Mechaduino is configured to start in closed loop mode.  
  setupSPI();                       // Sets up SPI for communicating with encoder
  digitalWrite(ledPin,LOW);         // turn LED off 
  SerialUSB.println("Start");
  mode = 'v';                       // start in velocity mode
}

void loop()                 // main loop
{
  serialCheck_2wire(); // check for machine to machine control, uart (rx, tx)
  serialCheck_usb(); // intended for human to machine control uart (usb)
}
