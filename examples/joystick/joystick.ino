/*
 joystick

 Simple sketch that show how to use the joystick of
 the DFRobot Accessory Shield
 
 created 3 Sept 2016
 by Biagio Montaruli

 this code is in the public domain
 */
#include <AccessoryShield.h>

// create two new variables to store the previous joystick and
// the new joystick value 
JoystickMode joystickOldValue, joystickNewValue;

void setup() {
  // Initialize Serial comunication
  Serial.begin(115200);
  // wait for the serial port to connect, Needed for USB native only
  while(!Serial) ;
  // initialize the Accessory Shield library
  accessoryShield.begin();
  // store the initial value of the joystick
  joystickOldValue = accessoryShield.getJoystickValue();
}

void loop() {
  // get the new joystick value
  joystickNewValue = accessoryShield.getJoystickValue();

  // if the old value is different from the previous value, print
  // the new joystick value.
  // NOTE : when the joystick is not moved the method getJoystickValue()
  // returns JOYSTICK_NONE but when you move the joystick, you read
  // a new value that is different from the prevoius.
  // For boards that use 3,3 V as voltage reference JOYSTICK_DOWN
  // as the same value of JOYSTICK_NONE and so the next switch()
  // construct use the case label JOYSTICK_NONE_OR_DOWN
  if(joystickNewValue != joystickOldValue) {
    switch(joystickNewValue) {
    #if (defined(_VARIANT_ARDUINO_101_X_) || defined(__SAM3X8E__) || defined(ARDUINO_ARCH_SAMD))
      case JOYSTICK_NONE_OR_DOWN:
        Serial.println("Joystick not moved or moved down");
        break;
    #else
      case JOYSTICK_NONE:
        Serial.println("Joystick not moved");
        break;
      case JOYSTICK_DOWN:
        Serial.println("Joystick moved down");
        break;
    #endif
      case JOYSTICK_LEFT:
        Serial.println("Joystick moved left");
        break;
      case JOYSTICK_RIGHT:
        Serial.println("Joystick moved right");
        break;
      case JOYSTICK_UP:
        Serial.println("Joystick moved up");
        break;
      case JOYSTICK_PUSH:
        Serial.println("Joystick pushed");
        break;
    }
    // update the old joystick value
    joystickOldValue = joystickNewValue;
  }
  // wait for a small delay
  delay(10);
}
