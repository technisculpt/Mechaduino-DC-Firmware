//Contains the Mechaduino parameter defintions

#include <Wire.h>
#include "Parameters.h"
#include "math.h"


//----Current Parameters-----

volatile float Fs = 6500.0;   //Sample frequency in Hz

volatile float pKp = 0.14;      //position mode PID values.  Depending on your motor/load/desired performance, you will need to tune these values.  You can also implement your own control scheme
volatile float pKi = 0.03;
volatile float pKd = 0.2;
volatile float pLPF = 30;       //break frequency in hertz
volatile float pI_limit = 70;

volatile float vKp = 20;       //velocity mode PID values.  Depending on your motor/load/desired performance, you will need to tune these values.  You can also implement your own control scheme
volatile float vKi = 0.0005;
volatile float vKd = 0.0;
volatile float vLPF = 100.0;       //break frequency in hertz

volatile float pLPFa = exp(pLPF*-2*3.14159/Fs); // z = e^st pole mapping
volatile float pLPFb = (1.0-pLPFa);
volatile float vLPFa = exp(vLPF*-2*3.14159/Fs); // z = e^st pole mapping
volatile float vLPFb = (1.0-vLPFa)* Fs * 0.16666667;