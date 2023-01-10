//Contains TC5 Controller definition
//The main control loop is executed by the TC5 timer interrupt:

#include <SPI.h>

#include "State.h"
#include "Utils.h"
#include "Parameters.h"
#include "analogFastWrite.h"

void TC5_Handler() {  // gets called with FPID frequency
  
  if (TC5->COUNT16.INTFLAG.bit.OVF == 1) {    // A counter overflow caused the interrupt
     
    //TEST1_HIGH();  //digitalWrite(3, HIGH);  //Fast Write to Digital 3 for debugging

    y = readEncoder()*0.02197399743; // 360/2^14
   
    if ((y - y_1) < -180.0) wrap_count += 1;  //Check if we've rotated more than a full revolution (have we "wrapped" around from 359 degrees to 0 or ffrom 0 to 359?)
    else if ((y - y_1) > 180.0) wrap_count -= 1;

    yw = (y + (360.0 * wrap_count));  //yw is the wrapped angle (can exceed one revolution)


    switch (mode) {
      case 'x':         // position control                        
          e = (r - yw);
        
          ITerm += (pKi * e);                             //Integral wind up limit
          if (ITerm > pI_limit) ITerm = pI_limit;
          else if (ITerm < -pI_limit) ITerm = -pI_limit;          
          
          DTerm = pLPFa*DTerm -  pLPFb*pKd*(yw-yw_1);
          
          u = (pKp * e) + ITerm + DTerm;

          break;

      case 'v':         // velocity controler
        v = vLPFa*v +  vLPFb*(yw-yw_1);  //filtered velocity called "DTerm" because it is similar to derivative action in position loop

        e = (r - v);   //error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )

        ITerm += (vKi * e);  //Integral wind up limit
        if (ITerm > 200) ITerm = 200;
        else if (ITerm < -200) ITerm = -200;
      
        u = ((vKp * e) + ITerm - (vKd * (e-e_1)));

        break;

    }

    if (u > 0)  {
      IN_2_HIGH();  //REG_PORT_OUTSET0 = PORT_PA21;     //write IN_2 HIGH
      IN_1_LOW();   //REG_PORT_OUTCLR0 = PORT_PA06;     //write IN_1 LOW
    }
    else  {
      IN_2_LOW();   //REG_PORT_OUTCLR0 = PORT_PA21;     //write IN_2 LOW
      IN_1_HIGH();  //REG_PORT_OUTSET0 = PORT_PA06;     //write IN_1 HIGH
    }
    
    analogFastWrite(VREF_1, int(abs(u)));

    if (u > 0){                 
      if (abs(e) < 0.1) ledPin_HIGH();    // turn on LED if error is less than 0.1
      else ledPin_LOW();                  //digitalWrite(ledPin, LOW);
    }

    y_1 = y;
    yw_1 = yw;
    e_1 = e;

    TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    //TEST1_LOW();            //for testing the control loop timing
  }
}








