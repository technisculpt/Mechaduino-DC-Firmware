//Contains the declaration of the state variables for the control loop  

#ifndef __STATE_H__
#define __STATE_H__


//interrupt vars
extern volatile bool i_enabled;

extern volatile float r;  //setpoint
extern volatile float y;  // measured angle
extern volatile float v;  // estimated velocity (velocity loop)
extern volatile float yw;
extern volatile float yw_1;
extern volatile float e;  // e = r-y (error)
extern volatile float e_1;  // e = r-y (error)
extern volatile float p;  // proportional effort
extern volatile float i;  // integral effort

extern volatile float u;  // control effort

extern volatile long wrap_count;
extern volatile float y_1;

extern volatile float ITerm;
extern volatile float DTerm;
extern char mode;

#endif








