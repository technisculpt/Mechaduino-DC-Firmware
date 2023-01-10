#include <Esplora.h>

/*

Esplora Firmware to control a Mechaduino using the Serial1 UART channel (pins PD2 & PD3 are RXD1 and RXD2)
The Serial class (USB UART) can still be used for debugging purposes
Mark Lagana 10/1/23

* INPUT | COMMAND (Velocity in RPM) currently no position mode
-----------------
up button : clockwise at predefined velocity set by set_point variable
down button : anti-clockwise at predefined velocity set by set_point variable
left button : clockwise at velocity set by the Esplora slider
right button : anti-clockwise at velocity set by the Esplora slider
joystick up : (closed loop) clockwise at velocity set by joystick position 
joystick down : (closed loop) anti-clockwise at velocity set by joystick position
joystick right : (open loop) clockwise at velocity set by joystick position 
joystick left : (open loop) anti-clockwise at velocity set by joystick position
-----------------

frame structure:

char command | velocity | terminator ('q')

commands:
d - clockwise (closed loop)
u - anti clockwise (closed loop)
x - clockwise (open loop)
z - anti clockwise (open loop)
s - stop

*/

bool joy_mode_1 = false;
bool joy_mode_2 = false;
int joystickDebounce = 30;
byte set_point = 15; // RPM 
int min_speed = 5;
int max_speed = 100;

void setup() {
  Serial1.begin(115200);
}

void loop() {
  Esplora.writeRGB(0, 0, 0);
  bool d_button = Esplora.readButton(SWITCH_DOWN);
  bool u_button = Esplora.readButton(SWITCH_UP);
  bool l_button = Esplora.readButton(SWITCH_LEFT);
  bool r_button = Esplora.readButton(SWITCH_RIGHT);
  bool j_button = Esplora.readJoystickButton();
  int y_value = Esplora.readJoystickY();
  int x_value = Esplora.readJoystickX();
  int s_value = Esplora.readSlider();

  if(d_button == LOW) // go up at predefined RPM setpoint
  {
    Serial1.print('d' + String(set_point) + 'q');
    Esplora.writeRGB(128, 128, 128);
  }

  if(u_button == LOW) // go down at predefined RPM setpoint
  {
    Serial1.print('u' + String(set_point) + 'q');
    Esplora.writeRGB(128, 128, 128);
  }

  if(l_button == LOW) // go up at slider defined RPM setpoint
  {
    byte rpm = map(s_value, 0, 1023, min_speed, max_speed);
    Serial1.print('d' + String(rpm) + 'q');
    Esplora.writeRGB(128, 128, 128);
  }

  if(r_button == LOW) // go down at slider defined RPM setpoint
  {
    byte rpm = map(s_value, 0, 1023, min_speed, max_speed);
    Serial1.print('u' + String(rpm) + 'q');
    Esplora.writeRGB(128, 128, 128);
  }

  if(j_button == LOW) // s stop
  {
    Serial1.print('s' + String(0) + 'q');
    Esplora.writeRGB(128, 128, 128);
  }

  if(y_value > joystickDebounce){ // go up at joystick y defined RPM setpoint
    byte rpm = map(y_value, 0, 512, min_speed, max_speed);
    Serial1.print('d' + String(rpm) + 'q');
    joy_mode_1 = true;
    Esplora.writeRGB(128, 128, 128);
    delay(100);
  }

  if(y_value < -joystickDebounce){  // down at joystick y defined RPM setpoint
    byte rpm = map(y_value, 0, -512, min_speed, max_speed);
    Serial1.print('u' + String(rpm) + 'q');
    joy_mode_1 = true;
    Esplora.writeRGB(128, 128, 128);
    delay(100);
  }

  if(x_value > joystickDebounce){ // go up at joystick y defined open loop voltage
    byte rpm = map(x_value, 0, 512, min_speed, max_speed);
    Serial1.print('x' + String(rpm) + 'q');
    joy_mode_2 = true;
    Esplora.writeRGB(128, 128, 128);
    delay(100);
  }

  if(x_value < -joystickDebounce){ // go down at joystick y defined open loop voltage
    byte rpm = map(x_value, 0, -512, min_speed, max_speed);
    Serial1.print('z' + String(rpm) + 'q');
    joy_mode_2 = true;
    Esplora.writeRGB(128, 128, 128);
    delay(100);
  }


  if(joy_mode_1 && (y_value < joystickDebounce) && (y_value > -joystickDebounce)){ // s stop if joystick relaxed
      joy_mode_1 = false;
      Serial1.print('s' + String(0) + 'q');
      Esplora.writeRGB(128, 128, 128);
  }

  if(joy_mode_2 && (x_value < joystickDebounce) && (x_value > -joystickDebounce)){ // s stop if joystick relaxed
      joy_mode_2 = false;
      Serial1.print('s' + String(0) + 'q');
      Esplora.writeRGB(128, 128, 128);
  }

  delay(200);
  
}