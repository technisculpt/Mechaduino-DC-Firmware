//Contains the definitions of the functions used by the firmware.


#include <SPI.h>
#include <Wire.h>

#include "Parameters.h"
#include "Controller.h"
#include "Utils.h"
#include "State.h"
#include "analogFastWrite.h"

void setupPins() {

  pinMode(VREF_2, OUTPUT);
  pinMode(VREF_1, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_1, OUTPUT);

  pinMode(chipSelectPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer

  
#ifdef ENABLE_PROFILE_IO  
  pinMode(TEST1, OUTPUT);
#endif

  pinMode(ledPin, OUTPUT); //

  // pinMode(clockPin, OUTPUT); // SCL    for I2C
  // pinMode(inputPin, INPUT); // SDA

  IN_2_HIGH();   //  digitalWrite(IN_2, HIGH);
  IN_1_LOW();    //  digitalWrite(IN_1, LOW);
}

void setupSPI() {
  SPISettings settingsA(10000000, MSBFIRST, SPI_MODE1);             ///400000, MSBFIRST, SPI_MODE1);
  SPI.begin();    //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
  delay(1000);
  SPI.beginTransaction(settingsA);
}

void serialCheck_2wire() {
    
  if (Serial1.available()) {

    String incoming = Serial1.readStringUntil('q');
    char command = (char)incoming[0];
    incoming.remove(0, 1);
    int vel = incoming.toInt();

    switch (command) {

      case 'd': // d go up at predefined RPM setpoint
        r = vel;
        enableTCInterrupts();
        break;
        
      case 'u': // u go down at predefined RPM setpoint
        r = -vel;
        enableTCInterrupts();
        break;        

      case 's': // s stop
        disableTCInterrupts(); //disable closed loop
        analogFastWrite(VREF_1, 0);                       
        break;

      case 'x': // x go up at joystick y defined open loop voltage
        disableTCInterrupts(); //disable closed loop
        IN_2_HIGH();
        IN_1_LOW();
        analogFastWrite(VREF_1, vel);  
        break;

      case 'z': // z go down at joystick y defined open loop voltage
        disableTCInterrupts(); //disable closed loop
        IN_2_LOW();
        IN_1_HIGH();
        analogFastWrite(VREF_1, vel);                     
        break;

      default:
        break;
    }
  }
}

void serialCheck_usb() {        //Monitors uart1 (rx/tx) for commands.  Must be called in routinely in loop for serial interface to work.

  if (SerialUSB.available()) {

    char inChar = (char)SerialUSB.read();

    switch (inChar) {

      case 'p':             //print
        print_angle();
        break;

      case 'e':
        readEncoderDiagnostics();   //encoder error?
        break;

      case 'y':
        r = (readEncoder()*0.02197399743+(360.0 * wrap_count)); // hold the current position
        SerialUSB.print("New setpoint ");
        SerialUSB.println(r, 2);
        enableTCInterrupts();      //enable closed loop
        break;

      case 'n':
        disableTCInterrupts();      //disable closed loop
        analogFastWrite(VREF_2, 0);     //set phase currents to zero
        analogFastWrite(VREF_1, 0);                       
        break;

      case 'r':             //new setpoint
        SerialUSB.println("Enter setpoint:");
        while (SerialUSB.available() == 0)  {}
        r = SerialUSB.parseFloat();
        SerialUSB.println(r);
        break;

      case 'x':
        mode = 'x';           //position loop
        break;

      case 'v':
        mode = 'v';           //velocity loop
        break;
 
      case 'q':
        parameterQuery();     // prints copy-able parameters
        break;

      case 'k':
        parameterEditmain();
        break;

      case 'm':
        serialMenu();
        break;

      default:
        break;
    }
  }
}

void print_angle()                ///////////////////////////////////       PRINT_ANGLE   /////////////////////////////////
{
  SerialUSB.print("angle: ");
  SerialUSB.print(y);
  SerialUSB.print(", raw encoder: ");
  SerialUSB.print(readEncoder());
  SerialUSB.println();
}


void parameterQuery() {         //print current parameters in a format that can be copied directly in to Parameters.cpp
  SerialUSB.println(' ');
  SerialUSB.println("----Current Parameters-----");
  SerialUSB.println(' ');
  SerialUSB.println(' ');

  SerialUSB.print("volatile float Fs = ");
  SerialUSB.print(Fs, DEC);
  SerialUSB.println(";  //Sample frequency in Hz");
  SerialUSB.println(' ');

  SerialUSB.print("volatile float pKp = ");
  SerialUSB.print(pKp, DEC);
  SerialUSB.println(";      //position mode PID vallues.");
  
  SerialUSB.print("volatile float pKi = ");
  SerialUSB.print(pKi, DEC);
  SerialUSB.println(";");

  SerialUSB.print("volatile float pKd = ");
  SerialUSB.print(pKd, DEC);
  SerialUSB.println(";");
  
  SerialUSB.print("volatile float pLPF = ");
  SerialUSB.print(pLPF, DEC);
  SerialUSB.println(";");

  SerialUSB.println(' ');

  SerialUSB.print("volatile float vKp = ");
  SerialUSB.print(vKp, DEC);
  SerialUSB.println(";      //velocity mode PID vallues.");

  SerialUSB.print("volatile float vKi = ");
  SerialUSB.print(vKi , DEC);
  SerialUSB.println(";");

  SerialUSB.print("volatile float vKd = ");
  SerialUSB.print(vKd, DEC);
  SerialUSB.println(";");
  SerialUSB.print("volatile float vLPF = ");
  SerialUSB.print(vLPF, DEC);
  SerialUSB.println(";");
}

void parameterEditmain() {

  SerialUSB.println();
  SerialUSB.println("Edit parameters:");
  SerialUSB.println();
  SerialUSB.println("p ----- position loop");
  SerialUSB.println("v ----- velocity loop");
  SerialUSB.println("q ----- quit");
  SerialUSB.println();

  while (SerialUSB.available() == 0)  {}
  char inChar2 = (char)SerialUSB.read();

  switch (inChar2) {
    case 'p':
      {
        parameterEditp();
      }
      break;

    case 'v':
      {
        parameterEditv();
      }
      break;

    default:
      {}
      break;
  }
}

void parameterEditp() {
  
  bool quit = false;
  while(!quit){
    SerialUSB.println("Edit position loop gains:");
    SerialUSB.println();
    SerialUSB.print("p ----- pKp = ");
    SerialUSB.println(pKp, DEC);
    SerialUSB.print("i ----- pKi = ");
    SerialUSB.println(pKi, DEC);
    SerialUSB.print("d ----- pKd = ");
    SerialUSB.println(pKd, DEC);
    SerialUSB.print("l----- LPF = ");
    SerialUSB.println(pLPF,DEC);
    SerialUSB.println("q ----- quit");
    SerialUSB.println();
    
    while (SerialUSB.available() == 0)  {}
    char inChar3 = (char)SerialUSB.read();
    
    switch (inChar3) {
      case 'p':
        {
          SerialUSB.println("pKp = ?");
          while (SerialUSB.available() == 0)  {}
          pKp = SerialUSB.parseFloat();
          SerialUSB.print("new pKp = ");
          SerialUSB.println(pKp, DEC);
          SerialUSB.println("");
        }
        break;
      case 'i':
        {
          SerialUSB.println("pKi = ?");
          while (SerialUSB.available() == 0)  {}
          pKi = SerialUSB.parseFloat();
          SerialUSB.print("new pKi = ");
          SerialUSB.println(pKi, DEC);
          SerialUSB.println("");
        }
        break;
      case 'd':
        {
          SerialUSB.println("pKd = ?");
          while (SerialUSB.available() == 0)  {}
          pKd = SerialUSB.parseFloat();
          SerialUSB.print("new pKd = ");
          SerialUSB.println(pKd, DEC);
          SerialUSB.println("");
        }
        break;
       case 'l':
        {
          SerialUSB.println("pLPF = ?");
          while (SerialUSB.available() == 0)  {}
          pLPF = SerialUSB.parseFloat();
          pLPFa = exp(pLPF*-2*3.14159/Fs);
          pLPFb = (1.0-pLPFa);
          SerialUSB.print("new pLPF = ");
          SerialUSB.println(pLPF, DEC);
          SerialUSB.println("");
        }
        break;
      case 'q':
        {  
          quit = true;
          SerialUSB.println("");
          SerialUSB.println("done...");
          SerialUSB.println("");
        }
      default:
        {}
        break;
    }
  }
}

void parameterEditv() {
  bool quit = false;
  while(!quit){  
    SerialUSB.println("Edit velocity loop gains:");
    SerialUSB.println();
    SerialUSB.print("p ----- vKp = ");
    SerialUSB.println(vKp, DEC);
    SerialUSB.print("i ----- vKi = ");
    SerialUSB.println(vKi, DEC);
    SerialUSB.print("d ----- vKd = ");
    SerialUSB.println(vKd, DEC);
    SerialUSB.print("l ----- vLPF = ");
    SerialUSB.println(vLPF, DEC);
    SerialUSB.println("q ----- quit");
    SerialUSB.println();
  
    while (SerialUSB.available() == 0)  {}
    char inChar4 = (char)SerialUSB.read();
  
    switch (inChar4) {
      case 'p':
        {
          SerialUSB.println("vKp = ?");
          while (SerialUSB.available() == 0)  {}
          vKp = SerialUSB.parseFloat();
          SerialUSB.print("new vKp = ");
          SerialUSB.println(vKp, DEC);
        }
        break;
      case 'i':
        {
          SerialUSB.println("vKi = ?");
          while (SerialUSB.available() == 0)  {}
          vKi = SerialUSB.parseFloat();
          SerialUSB.print("new vKi = ");
          SerialUSB.println(vKi, DEC);
        }
        break;
      case 'd':
        {
          SerialUSB.println("vKd = ?");
          while (SerialUSB.available() == 0)  {}
          vKd = SerialUSB.parseFloat();
          SerialUSB.print("new vKd = ");
          SerialUSB.println(vKd, DEC);
        }
        break;
       case 'l':
        {
          SerialUSB.println("vLPF = ?");
          while (SerialUSB.available() == 0)  {}
          vLPF = SerialUSB.parseFloat();
          vLPFa = (exp(vLPF*-2*3.14159/Fs));
          vLPFb = (1.0-vLPFa)* Fs * 0.16666667;
          SerialUSB.print("new vLPF = ");
          SerialUSB.println(vLPF, DEC);
          SerialUSB.println("");
        }
        break;
      case 'q':
        {  
          quit = true;
          SerialUSB.println("");
          SerialUSB.println("done...");
          SerialUSB.println("");  
        }
      default:
        {}
        break;
    }
  }  
}

void serialMenu() {
  SerialUSB.println("");
  SerialUSB.println("");
  SerialUSB.println("----- Mechaduino_DC -----");
  SerialUSB.print("Firmware: ");
  SerialUSB.println(firmware_version);
  SerialUSB.print("Identifier: ");
  SerialUSB.println(identifier);
  SerialUSB.println("");
  SerialUSB.println("Main menu");
  SerialUSB.println("");
  SerialUSB.println(" p  -  print angle");
  SerialUSB.println("");
  SerialUSB.println(" e  -  check encoder diagnositics");
  SerialUSB.println(" q  -  parameter query");
  SerialUSB.println("");
  SerialUSB.println(" x  -  position mode");
  SerialUSB.println(" v  -  velocity mode");
  SerialUSB.println("");
  SerialUSB.println(" y  -  enable control loop");
  SerialUSB.println(" n  -  disable control loop");
  SerialUSB.println(" r  -  enter new setpoint");
  SerialUSB.println("");
  SerialUSB.println(" k  -  edit controller gains -- note, these edits are stored in volatile memory and will be reset if power is cycled");
  SerialUSB.println(" m  -  print main menu");
  // SerialUSB.println(" f  -  get max loop frequency");
  SerialUSB.println("");
}

int readEncoder()
{
  long angleTemp;
  
  CHIPSELECT_LOW(); //digitalWrite(chipSelectPin, LOW);

  byte b1 = SPI.transfer(0xFF);
  byte b2 = SPI.transfer(0xFF);

  angleTemp = (((b1 << 8) | b2) & 0B0011111111111111);

  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  return angleTemp;
}

void readEncoderDiagnostics()
{
  long angleTemp;
  CHIPSELECT_LOW(); //digitalWrite(chipSelectPin, LOW);

  ///////////////////////////////////////////////READ DIAAGC (0x3FFC)
  SerialUSB.println("------------------------------------------------");

  SerialUSB.println("Checking AS5047 diagnostic and error registers");
  SerialUSB.println("See AS5047 datasheet for details");
  SerialUSB.println(" ");
  ;

  SPI.transfer(0xFF);
  SPI.transfer(0xFC);

  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  delay(1);
  CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

  byte b1 = SPI.transfer(0xC0);
  byte b2 = SPI.transfer(0x00);

  SerialUSB.print("Check DIAAGC register (0x3FFC) ...  ");
  SerialUSB.println(" ");

  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.println((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14))    SerialUSB.println("  Error occurred  ");

  if (angleTemp & (1 << 11))    SerialUSB.println("  MAGH - magnetic field strength too high, set if AGC = 0x00. This indicates the non-linearity error may be increased");

  if (angleTemp & (1 << 10))    SerialUSB.println("  MAGL - magnetic field strength too low, set if AGC = 0xFF. This indicates the output noise of the measured angle may be increased");

  if (angleTemp & (1 << 9))     SerialUSB.println("  COF - CORDIC overflow. This indicates the measured angle is not reliable");

  if (angleTemp & (1 << 8))     SerialUSB.println("  LF - offset compensation completed. At power-up, an internal offset compensation procedure is started, and this bit is set when the procedure is completed");

  if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 11)) | (angleTemp & (1 << 10)) | (angleTemp & (1 << 9))))  SerialUSB.println("Looks good!");
  SerialUSB.println(" ");


  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  delay(1);
  CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

  SPI.transfer(0x40);
  SPI.transfer(0x01);
  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);

  delay(1);
  CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

  b1 = SPI.transfer(0xC0);
  b2 = SPI.transfer(0x00);


  SerialUSB.print("Check ERRFL register (0x0001) ...  ");
  SerialUSB.println(" ");



  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.println((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14)) {
    SerialUSB.println("  Error occurred  ");
  }
  if (angleTemp & (1 << 2)) {
    SerialUSB.println("  parity error ");
  }
  if (angleTemp & (1 << 1)) {
    SerialUSB.println("  invalid register  ");
  }
  if (angleTemp & (1 << 0)) {
    SerialUSB.println("  framing error  ");
  }
  if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 2)) | (angleTemp & (1 << 1)) | (angleTemp & (1 << 0))))  SerialUSB.println("Looks good!");

  SerialUSB.println(" ");

  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);

  delay(1);

}

void receiveEvent(int howMany)
{
  while (1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    SerialUSB.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  SerialUSB.println(x);         // print the integer
  r = 0.1 * ((float)x);
}

void setupTCInterrupts() {  // configure the controller interrupt

  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CC[0].reg = (int)( round(48000000 / Fs)); //0x3E72; //0x4AF0;
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  NVIC_SetPriority(TC5_IRQn, 1);              //Set interrupt priority

  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);

}

void enableTCInterrupts() {   //enables the controller interrupt ("closed loop mode")
  if(!i_enabled){
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
    WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
    i_enabled = true;
  }

}

void disableTCInterrupts() {  //disables the controller interrupt ("closed loop mode")
  if(i_enabled){
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
    WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
    i_enabled = false;
  }
}

