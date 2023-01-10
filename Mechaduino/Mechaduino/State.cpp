  //Contains the declaration of the state variables for the control loop  


//interrupt vars
volatile bool i_enabled = false;

volatile float r = 0.0;   //setpoint
volatile float y = 0.0;   // measured angle
volatile float v = 0.0;  // estimated velocity  (velocity loop)
volatile float yw = 0.0;  // "wrapped" angle (not limited to 0-360)
volatile float yw_1 = 0.0;
volatile float e = 0.0;   // e = r-y (error)
volatile float e_1 = 0.0;
volatile float p = 0.0;   // proportional effort
volatile float i = 0.0;   // integral effort

volatile float u = 0.0;     //control effort


volatile long wrap_count = 0;  //keeps track of how many revolutions the motor has gone though (so you can command angles outside of 0-360)
volatile float y_1 = 0;

volatile float ITerm = 0.0;
volatile float DTerm;

char mode;

