//Contains the declarations for the functions used by the firmware

#ifndef __UTILS_H__
#define __UTIL_H__

	void setupPins();                 // initializes pins
	
	void setupSPI();                  //initializes SPI

  	void enableInterrupt();           //enable pin interrupt handler

	void serialCheck_2wire();         //checks uart1 serial port for commands.  Must include this in loop() for serial interface to work

	void serialCheck_usb();           //checks usb serial port for commands.  Must include this in loop() for serial interface to work

	void print_angle();               //for debigging purposes in open loop mode:  prints [step number] , [encoder reading]

	void parameterQuery();

	void parameterEditmain();

	void parameterEditp();

	void parameterEditv();

	void serialMenu();

	int readEncoder();                //read raw encoder position
	  
	void readEncoderDiagnostics();    //check encoder diagnostics registers
		
	void receiveEvent(int howMany);   //for i2c interface...

	void setupTCInterrupts();         //configures control loop interrupt
	
	void enableTCInterrupts();        //enables control loop interrupt.  Use this to enable "closed-loop" modes
	
	void disableTCInterrupts();       //disables control loop interrupt.  Use this to diable "closed-loop" mode

#endif







