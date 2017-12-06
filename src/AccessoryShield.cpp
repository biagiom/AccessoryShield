/*
 ####################################################################################
 #  Accessory Shield Library for Arduino/Genuino boards                             #
 #  Copyright (C) 2017 Biagio Montaruli                                             #
 #  Oled library based on Adafruit_SSD1306 written by Limor Fried/Ladyada           #
 ####################################################################################
 #                                                                                  #
 #  AccessoryShield.cpp                                                             #
 #  This file is part of Accessory Shield Library.                                  #
 #                                                                                  #
 #  This library is free software: you can redistribute it and/or modify            #
 #  it under the terms of the GNU Lesser General Public License as published by     #
 #  the Free Software Foundation, either version 3 of the License, or               #
 #  (at your option) any later version.                                             #
 #                                                                                  #
 #  AccessoryShield library is distributed in the hope that it will be useful,      #
 #  but WITHOUT ANY WARRANTY; without even the implied warranty of                  #
 #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                            #
 #  See the GNU Lesser General Public License for more details.                     #
 #                                                                                  #
 #  You should have received a copy of the GNU Lesser General Public License        #
 #  along with this program.  If not, see <http://www.gnu.org/licenses/>.           #
 #                                                                                  #
 ####################################################################################
 */

#include "AccessoryShield.h"

/** joystickModes : array of strings
 *  @brief Array of strings that represent the joystick states
 */
#if (defined(_VARIANT_ARDUINO_101_X_) || defined(__SAM3X8E__) || defined(ARDUINO_ARCH_SAMD))
    const char *AccessoryShield::joystickModes[] = {"NONE or DOWN", "UP", "LEFT", "RIGHT", "PUSHED"};
#else
    const char *AccessoryShield::joystickModes[] = {"NONE", "UP", "DOWN", "LEFT", "RIGHT", "PUSHED"};
#endif

/**************************************** CONSTRUCTOR *******************************************/

/** AccessoryShield() : Constructor of AccessoryShield class
 *  @brief Constructor to initialize the data members of AccessoryShield class
 *  @param void
 */
AccessoryShield::AccessoryShield(void) {

    // set RGB led state => OFF
    for(uint8_t i = 0; i < numLeds; i++) {
        RGBstate[i] = LED_OFF;
    }
    
    for(uint8_t i = 0; i < 5; i++) {
        DHT11data[i] = 0;
    }
    
    // Set starting values for variables used to store temperature and humidity
    tempC = tempF = tempK = NAN;
    relHumidity = NAN;
    
	// buzzer initial state is LOW (buzzer is not active)
    buzzerState = LOW;
    
	// set joystick's initial value and state
    joystickMoved = false;
    joystickValue = 1023;
#if (defined(_VARIANT_ARDUINO_101_X_) || defined(__SAM3X8E__) || defined(ARDUINO_ARCH_SAMD))
    joystickState = JOYSTICK_NONE_OR_DOWN;
#else
    joystickState = JOYSTICK_NONE;
#endif

    // set relay's initial state
    relayState = LOW;
}

/**************************************** PRIVATE METHODS ****************************************/

/** checkPulse()
 *  @brief Private function member to check if the DHT11 data pin 2 stays on the specified state
 *         for a certain amount of time
 *  @param state: the pin state (HIGH or LOW)
 *  @return true or false according to the pin state read with digitalRead()
 */
bool AccessoryShield::checkPulse(bool state) const {
    uint16_t loopCnt = 10000;
    while(digitalRead(DHT11pin) == state)
        if (loopCnt-- == 0) 
            return DHT11_TIMEOUT_ERROR;

    return DHT11_ACK_RECEIVED;
}

/** getDHT11data()
 *  @brief Private function member to read temperature and humidity from the DHT11 sensor 
 *         attached to digital pin 2
 *  @param void
 *  @return one of the DHT11 read state represented by one of the values of DHT11State enum :
 *          DHT11_CHECKSUM_ERROR, DHT11_GETTING_DATA_ERROR, DHT11_TIMEOUT_ERROR,
 *          DHT11_ACK_RECEIVED, DHT11_DATA_READ
 */
int8_t AccessoryShield::getDHT11Data(void) {
    // bit counter
    uint8_t count = 7;
    // index for DHT11data[] array
    uint8_t dataIndex = 0;

    // reset the previous 5 byte data to zero
    for (uint8_t i = 0; i < 5; i++)
        DHT11data[i] = 0;

    delay(1000);

    // To start the communication between with DHT11, 
    // Arduino will set Data Single-bus voltage level from high to low
    // and this process must take at least 18ms to ensure DHT’s detection of Arduino's signal, 
    // then Arduino will pull up voltage and wait 20-40us for DHT’s response.
    pinMode(DHT11pin, OUTPUT);
    digitalWrite(DHT11pin, LOW);
    delay(18);
    digitalWrite(DHT11pin, HIGH);
    delayMicroseconds(40);
    pinMode(DHT11pin, INPUT);
    
    // Once DHT detects the start signal, it will send out a low-voltage-level response signal, 
    // which lasts 80us (DHT11 ACKNOWLEDGE). 
    if(checkPulse(LOW)) {

        // Then DHT sets the data line level from low to high and keeps it for 80us
        // for DHT11’s preparation for sending data.
        if(checkPulse(HIGH)) {
            // Now read the 40 bits sent by the sensor
            // When DHT starts sending data to Arduino, every bit of data begins
            // with the 50us low-voltage-level followed by the high-voltage level.
            // The length of the high signal determines whether data bit is "0" or "1" :
			// if the high signal last ~26us, the bit sent out by DHT11 is "0"
            // else if the high signal last ~100us, the bit sent out by DHT11 is "1"
            for (uint8_t i = 0; i < 40; i++) {
                if(checkPulse(LOW)) {
                    unsigned long delayHigh = micros();
	
                    if(checkPulse(HIGH)) {
                    // if the DHT11 high signal lasts more than 40 seconds,
                    // the bit sent by DHT11 is "1"
                        if ((micros() - delayHigh) > 40)
                            // set the bit (which position is specified by 'count') as "1"
                            DHT11data[dataIndex] |= (1 << count);
                        else
                            // else set the bit as "0"
                            DHT11data[dataIndex] &= ~(1 << count);
                            
                        // DHT11 sends 5 bytes of data (40 bit) and for each byte of data,
                        // the 8 bits are sent starting from the most significant bit (MSB).
						// For this reason the 'count' counter is set to 7 and every time DHT11
                        // sends a new bit, 'count' is decremented to point the next bit.
                        // When 'count' becomes "0", this means that the transmission 
                        // of 1 byte of data is finished and so reset 'count' to "7" in order to
                        // point to the MSB of the next byte of data
                        if (count == 0) {
                            count = 7;
                            dataIndex++;
                        }
                        else {
                            count--;
                        }
                    }
                    else {
        			    return DHT11_TIMEOUT_ERROR;
                    }
                }
                else {
                    return DHT11_TIMEOUT_ERROR;
                }
            }
            // Check if the data sent by DHT11 sensor are valid:
            // the sum of the first 4 bytes must be equal to the value of the 5th byte
            // which is the checksum
            uint8_t checksum = DHT11data[0] + DHT11data[1] + DHT11data[2] + DHT11data[3];
            
            #if defined(SERIAL_DEBUG)
            // print the bytes value in decimal and hexadecimal base in the Serial monitor
            for(uint8_t i = 0; i < 5; i++) {
                Serial.print("DHT11read[");
                Serial.print(i);
                Serial.print("] = ");
                Serial.print(DHT11data[i], HEX);
                Serial.print(" (HEX) ");
                Serial.print(DHT11data[i], DEC);
                Serial.println(" (DEC) ");
            }
            // print the checksum value in decimal and hexadecimal base in the Serial monitor
            Serial.print("Checksum = ");
            Serial.print(checksum, HEX);
            Serial.print(" (HEX) ");
            Serial.print(checksum, DEC);
            Serial.println(" (DEC) ");
            #endif
            // if the data transmission is OK set the private data member with the temperature
            // and humidity value, then convert the temperature read by DHT11 in degrees Celsius
            // and Fareneith.
            if (DHT11data[4] == checksum) {
                // humidity value is the first byte sent by DHT11 sensor
                relHumidity = DHT11data[0]; 
                // temperature value (in degrees Celsius) is the third byte sent by DHT11 sensor
                tempC = DHT11data[2];
                // convert Celsius temperature in degrees Fareneith with the 
                // private method convertTempCtoF()
                tempF = convertTempCtoF(tempC);
                // convert Celsius temperature in degrees Fareneith with the 
                // private method convertTempCtoK()
                tempK = convertTempCtoK(tempC);
                
                #if defined(SERIAL_DEBUG)
                // print temperature in degrees Celsius in the Serial monitor
                Serial.print("TempC = ");
                Serial.println(tempC);
                // print temperature in degrees Fareneith in the Serial monitor
                Serial.print("TempF = ");
                Serial.println(tempF);
                // print temperature in degrees Kelvin in the Serial monitor
                Serial.print("TempK = ");
                Serial.println(tempK);
                // print humidity value in the Serial monitor
                Serial.print("Humidity = ");
                Serial.println(relHumidity);
                #endif
                return DHT11_DATA_READ;
            }
            else {
                // else if checksum doesn't match the sum of the first 4 bytes sent by DHT11
                // raise the ERROR_CHECKSUM
                return DHT11_CHECKSUM_ERROR;
            }

        }
        else {
            return DHT11_TIMEOUT_ERROR;
        }
    }
    else {
		return DHT11_TIMEOUT_ERROR;
    }
}


/**************************************** PUBLIC METHODS ****************************************/

/** begin()
 *  @brief Public function member to initialize the RGB led, OLED display, relay,
 *         DHT11 temperature and humidity sensor, buzzer, potentimeter and joystick
 *         of Accessory Shield
 *  @param void
 *  @return void
 */
void AccessoryShield::begin(void) {
    // Set the mode of RGB led pins and turn the three leds OFF 
    pinMode(RED_LED, OUTPUT);
    digitalWrite(RED_LED, HIGH);
    pinMode(GREEN_LED, OUTPUT);
    digitalWrite(GREEN_LED, HIGH);
    pinMode(BLUE_LED, OUTPUT);
    digitalWrite(BLUE_LED, HIGH);
    
    // Set the mode of the potentiometer pin as INPUT and disable the internal pull-up resistor
    pinMode(potPin, INPUT);
    digitalWrite(potPin, LOW);
    
    // Set the mode of the active buzzer pin as OUTPUT and set its initial state (LOW => disabled)
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);

    // set the mode of joystick pin as INPUT and disable the internal pull-up resistor
    pinMode(joystickPin, INPUT);
    digitalWrite(joystickPin, LOW);

    // set the mode of relay pin as OUTPUT and set its state to LOW : relay is OFF
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW);

#if !defined(__AVR_ATmega32U4__)
    // initialize the OLED display
    oledBegin();
#endif
}

/** end()
 *  @brief Public function to reset the sensors'state of Accessory Shield 
 *  @param void
 *  @return void
 */
void AccessoryShield::end(void) {
    // turn RGB led OFF
    setRGB(LED_OFF, LED_OFF, LED_OFF);
    // turn the buzzer OFF
    buzzerOFF();
    // disable the relay
    relayOFF();
#if !defined(__AVR_ATmega32U4__)
    // clear OLED display
    clearOledDisplay();
    oledPaint();
#endif
}

/** readPot()
 *  @brief Public function member to read the analog value of the pot trimmer
 *         attached to Accessory Shield pin A1.
 *         This function also stores the potentiometer value in the private
 *         data member 'potValue'
 *  @param void
 *  @return value read from the potentiometer using analogRead()
 */
uint16_t AccessoryShield::readPot(void) {
    static uint16_t potRead;
    // read the pot value withe analogRead which returns a value based on the
    // analog refernce settings.
    // If AREF = DEFAULT, analogRead() returns a value from 0 to 1023 
    potRead = analogRead(potPin);
    potValue = potRead;
    return potRead;
}

/** redON()
 *  @brief Public function member to turn the red led OFF and to update 
 *         its state in the private data member RGBState[] (array)
 *  @param void
 *  @return void
 */
void AccessoryShield::redON(void) {
    // turn the red led ON using analogWrite() since the red led's pin 9 is a PWM pin
    analogWrite(RED_LED, 0);
    RGBstate[r] = LED_ON;
}

/** redOFF()
 *  @brief Public function member to turn the red led OFF and to update 
 *         its state in the private data member RGBState[] (array)
 *  @param void
 *  @return void
 */
void AccessoryShield::redOFF(void) {
    // turn the red led OFF using analogWrite() since the red led's pin 9 is a PWM pin
    analogWrite(RED_LED, 255);
    RGBstate[r] = LED_OFF;
}

/** greenON()
 *  @brief Public function member to turn the green led ON and to update 
 *         its state in the private data member RGBState[] (array)
 *  @param void
 *  @return void
 */
void AccessoryShield::greenON(void) {
#if defined(_VARIANT_ARDUINO_101_X_)
    // Because of the green led's pin 10 is not a PWM pin, uses digitalWrite() to turn it ON
    digitalWrite(GREEN_LED, LOW);
#else
    // for the other boards uses analogWrite() to turn the green led ON
    analogWrite(GREEN_LED, 0);
#endif
    RGBstate[g] = LED_ON;
}

/** greenOFF()
 *  @brief Public function member to turn the green led OFF and to update 
 *         its state in the private data member RGBState[] (array)
 *  @param void
 *  @return void
 */
void AccessoryShield::greenOFF(void) {
#if defined(_VARIANT_ARDUINO_101_X_)
    // Because of the green led's pin 10 is not a PWM pin, uses digitalWrite() to turn it OFF
    digitalWrite(GREEN_LED, HIGH);
#else
    // for the other boards uses analogWrite() to turn the green led ON
    analogWrite(GREEN_LED, 255);
#endif
    RGBstate[g] = LED_OFF;
}

/** blueON()
 *  @brief Public function member to turn the blue led ON and to update 
 *         its state in the private data member RGBState[] (array)
 *  @param void
 *  @return void
 */
void AccessoryShield::blueON(void) {
    // turn the blue led ON using analogWrite() since the blue led's pin 3 is a PWM pin
    analogWrite(BLUE_LED, 0);
    RGBstate[b] = LED_ON;
}

/** blueOFF()
 *  @brief Public function member to turn the blue led OFF and to update 
 *         its state in the private data member RGBState[] (array)
 *  @param void
 *  @return void
 */
void AccessoryShield::blueOFF(void) {
    // turn blue led OFF using analogWrite() since the blue led's pin 3 is a PWM pin
    analogWrite(BLUE_LED, 255);
    RGBstate[b] = LED_OFF;
}

/** setRGB()
 *  @brief Public function member to set the RGB led state/brightness
 *         If using the Arduino/Genuino 101 with Accessory Shield, the green led state is
 *         set with the function digitalWrite() since the green led's pin 10 is not a PWM pin
 *         for Arduino/Genuino 101
 *  @param redState : value used to set the red led state/brightness with analogWrite()
 *  @param greenState : value used to set the green led state/brightness with analogWrite()
 *  @param blueState : value used to set the blue led state/brightness with analogWrite()
 *  @return void
 */
void AccessoryShield::setRGB(uint8_t redState, uint8_t greenState, uint8_t blueState) {
    // IMPORTANT NOTE :
    // The DFRobot Accessory Shield has a Common Anode RGB LED, so
    // when one of the three led pin is LOW the respective led is ON, 
    // instead when the led pin is HIGH the respective led is OFF
    RGBstate[r] = redState;
    analogWrite(RED_LED, 255 - redState);

#if defined(_VARIANT_ARDUINO_101_X_)
    if(greenState) {
        digitalWrite(GREEN_LED, LOW);
        RGBstate[g] = LED_ON;
    }
    else {
        digitalWrite(GREEN_LED, HIGH);
        RGBstate[g] = LED_OFF;
    }
#else
    RGBstate[g] = greenState;
    analogWrite(GREEN_LED, 255 - greenState);
#endif

    RGBstate[b] = blueState;
    analogWrite(BLUE_LED, 255 - blueState);
}

/** getRGBstate()
 *  @brief Public function member to get the RGB led state/brightness
 *  @param led : the led of which you want to know the state
 *  @return the led state (ON = 255, or OFF = 0, since the three leds are PWM leds)
 */
uint8_t AccessoryShield::getRGBstate(uint8_t led) {
    switch(led) {
    case RED_LED:
        return RGBstate[r];
        break;
    case GREEN_LED:
        return RGBstate[g];
        break;
    case BLUE_LED:
        return RGBstate[b];
        break;
    default:
        return RGBstate[r];
    }
}
	
/** buzzerON()
 *  @brief Public function member to turn the active buzzer ON
 *  @param void
 *  @return void
 */
inline void AccessoryShield::buzzerON(void) {
    digitalWrite(buzzerPin, HIGH);
    buzzerState = HIGH;
}

/** buzzerOFF()
 *  @brief Public function member to turn the active buzzer OFF
 *  @param void
 *  @return void
 */
void AccessoryShield::buzzerOFF(void) {
    digitalWrite(buzzerPin, LOW);
    buzzerState = LOW;
}

/** getBuzzerState()
 *  @brief Public function member to get the Buzzer state
 *  @param void
 *  @return HIGH if the buzzer is activated, or LOW if the buzzer is disabled
 */
bool AccessoryShield::getBuzzerState(void) const {
    return buzzerState;
}

/** playBuzzer()
 *  @brief Public function member to play with the buzzer attached to digital pin 8.
 *         Setting the frequency and the delay time between the moment the buzzer is ON
 *         and the moment the buzzer is OFF, the active buzzer plays a sound
 *  @param freq : number of times the active buzzer changes its state making a sound
 *  @param delayTime : delay time between buzzer activation (ON) and deactivation (OFF)
 *  @return void
 */
void AccessoryShield::playBuzzer(long freq, long delayTime) {
    if(freq < 0)
        freq = abs(freq);

    if(delayTime < 0)
        delayTime = abs(delayTime);

    for(long i = 0; i < freq; i++) {
        buzzerON();
        delay(delayTime);
        buzzerOFF();
        delay(delayTime);
    }
}

/** relayON()
 *  @brief Public function member to turn ON the relay attached to digital pin 11
 *  @param void
 *  @return void
 */
void AccessoryShield::relayON(void) {
    digitalWrite(relayPin, HIGH);
    relayState = HIGH;
}

/** relayOFF()
 *  @brief Public function member to turn OFF the relay attached to digital pin 11
 *  @param void
 *  @return void
 */
void AccessoryShield::relayOFF(void) {
    digitalWrite(relayPin, LOW);
    relayState = LOW;
}

/** getRelayState()
 *  @brief Public function member to get the Relay state
 *  @param void
 *  @return HIGH if the relay is activated, or LOW if the relay is disabled
 */
bool AccessoryShield::getRelayState(void) const {
    return relayState;
}

/** getJoystickValue()
 *  @brief Public function member to get the joystick state
 *  @param void
 *  @return joystick value represented by one of the values of JoystickMode enum :
 *          UP, DOWN, LEFT, RIGHT, NONE.  
 */
JoystickMode AccessoryShield::getJoystickValue(void) {
    uint16_t joystickRead;

    // Use the default Analog Reference for AVR boards and set analog resolution to 10
    // for boards based on SAMD and SAM architecture
    // TODO: possible improvement: add support for different Analog References
#if defined(__AVR__)
    analogReference(DEFAULT);
#elif defined(__SAM3X8E__) || defined(ARDUINO_ARCH_SAMD)
    analogReadResolution(10);
    analogWriteResolution(10);
#endif

    // read the joystick analog value from joystick pin A0
    joystickRead = analogRead(joystickPin);
#if defined(SERIAL_DEBUG)
    Serial.print("Analog value read from joystick : ");
    Serial.println(joystickRead);
#endif
	
    // When the joystick is not moved analogRead() returns 1023 which is also the
    // start value of the joystick, for this reason if the new value read with analogRead()
    // is different from the previous value the joystick hasn't been moved, instead
    // according to the read value set the respective joystick state
    if(joystickRead != joystickValue) {
        joystickMoved = true;

        if((joystickRead >= 0) && (joystickRead <= 5))
            joystickState = JOYSTICK_RIGHT;
#if (defined(ARDUINO_ARCH_ARC32) || defined(__SAM3X8E__) || defined(ARDUINO_ARCH_SAMD))
        else if((joystickRead >= 206) && (joystickRead <= 218))
            joystickState = JOYSTICK_PUSH;
        else if((joystickRead >= 470) && (joystickRead <= 493))
            joystickState = JOYSTICK_UP;
        else if((joystickRead >= 726) && (joystickRead <= 758))
            joystickState = JOYSTICK_LEFT;
        else if(joystickRead == 1023)
            joystickState = JOYSTICK_NONE_OR_DOWN;
#else
        else if((joystickRead >= 144) && (joystickRead <= 145))
            joystickState = JOYSTICK_PUSH;
        else if((joystickRead >= 329) && (joystickRead <= 330))
            joystickState = JOYSTICK_UP;
        else if((joystickRead >= 505) && (joystickRead <= 506))
            joystickState = JOYSTICK_LEFT;
        else if((joystickRead >= 741) && (joystickRead <= 743))
            joystickState = JOYSTICK_DOWN;
        else if(joystickRead == 1023)
            joystickState = JOYSTICK_NONE;
#endif
        // update the joystick value with the new value read with analogRead()
        joystickValue = joystickRead;
    }
    // return the joystick state
    return joystickState;
}

/** getJoystickStateStr()
 *  @brief Public function member to get a String representing the Joystick status
 *  @param void
 *  @return joystick status represented by an object of type String :
 *          "UP", "DOWN", "LEFT", "RIGHT", "NONE", "NONE or DOWN".
 */
String AccessoryShield::getJoystickStateStr(void) {
    return joystickModes[getJoystickValue()];
}

/** convertTempCtoF()
 *  @brief Public method that converts temperature in degrees Celsius 
 *         to temperature in degrees Fareneith
 *  @param tC : temperature in degrees Celsius
 *  @return temperature in degrees Fareneith
 */
float AccessoryShield::convertTempCtoF(float tC) {
    return (tC * 1.80) + 32.00;
}

/** convertTempCtoK()
 *  @brief Public method that converts temperature in degrees Celsius 
 *         to temperature in degrees Kelvin
 *  @param tC : temperature in degrees Celsius
 *  @return temperature in degrees Kelvin
 */
float AccessoryShield::convertTempCtoK(float tC) {
    return tC + 273.15;
}

/** convertTempFtoC()
 *  @brief Public method that converts temperature in degrees Fareneith 
 *         to temperature in degrees Celsius
 *  @param tF : temperature in degrees Fareneith
 *  @return temperature in degrees Celsius
 */
float AccessoryShield::convertTempFtoC(float tF) {
    return (tF - 32.00) / 1.80;
}

/** convertTempFtoK()
 *  @brief Public method that converts temperature in degrees Fareneith 
 *         to temperature in degrees Kelvin
 *  @param tF : temperature in degrees Fareneith
 *  @return temperature in degrees Kelvin
 */
float AccessoryShield::convertTempFtoK(float tF) {
    return (tF + 459.67) / 1.80;
}

/** convertTempKtoC()
 *  @brief Public method that converts temperature in degrees Kelvin 
 *         to temperature in degrees Celsius
 *  @param tK : temperature in degrees Kelvin
 *  @return temperature in degrees Celsius
 */
float AccessoryShield::convertTempKtoC(float tK) {
    return tK - 273.15;
}

/** convertTempKtoC()
 *  @brief Public method that converts temperature in degrees Kelvin 
 *         to temperature in degrees Fareneith
 *	@param tK : temperature in degrees Kelvin
 *  @return temperature in degrees Fareneith
 */
float AccessoryShield::convertTempKtoF(float tK) {
    return (tK * 1.80) - 459.67;
}

/** getTemperature()
 *  @brief Public method that calls the private method getDHT11Data() to get temperature 
 *         in the specified unit of measure (by default in degrees Celsius) 
 *  @param tempUnit : temperature unit
 *  @return temperature in the unit of measure specified by the parameter "tempUnit"
 *          or NAN if it can't get data from DHT11 sensor
 */
float AccessoryShield::getTemperature(TemperatureUnit tempUnit) {
    int8_t DHT11state = getDHT11Data();
    if(DHT11state == DHT11_DATA_READ) {
        switch(tempUnit) {
        case DHT11_TEMP_CELSIUS:
            return tempC;
            break;
        case DHT11_TEMP_FARENEITH:
            return tempF;
            break;
        case DHT11_TEMP_KELVIN:
            return tempK;
            break;
        default:
            return tempC;
        }
    }
    else {
#if defined(SERIAL_DEBUG)
        Serial.print("Error code = ");
        Serial.println(DHT11state);
#endif
        return NAN;
    }
}

/** getHumidity()
 *  @brief Public method that calls the private method getDHT11Data() to get relative humidity 
 *  @param void
 *  @return relative humidity or NAN if it can't get data from DHT11 sensor
 */
float AccessoryShield::getHumidity(void) {
    int8_t DHT11state = getDHT11Data();
    if(DHT11state == DHT11_DATA_READ) {
        return relHumidity;
    }
    else {
#if defined(SERIAL_DEBUG)
        Serial.print("Error code = ");
        Serial.println(DHT11state);
#endif
        return NAN;
    }		
}

/** getEnvironmentalData()
 *  @brief Public method that calls the get temperature and humidity value from DHT11 sensor and store
 *         these values in the references temp and hum 
 *  @param hum : relative humidity value
 *  @param temp : temperature value
 *  @param tUnit : unit of measure used to get temperature
 *  @return state of DHT11 sensor. If this method returns DHT11_DATA_READ, the environmental data
 *          have been read correctly from DHT11 sensor, otherwise it happened an error and in this case
 *          this method returns the error code
 */
int8_t AccessoryShield::getEnvironmentalData(float &hum, float &temp, TemperatureUnit tUnit) {
    int8_t DHT11state = getDHT11Data();
    if(DHT11state == DHT11_DATA_READ) {
        hum = relHumidity;

        switch(tUnit) {
        case DHT11_TEMP_CELSIUS:
            temp = tempC;
            break;
        case DHT11_TEMP_FARENEITH:
            temp = tempF;
            break;
        case DHT11_TEMP_KELVIN:
            temp = tempK;
            break;
        }
    }
#if defined(SERIAL_DEBUG)
    else {
        Serial.println("Warning : Impossible to get environmental data from DHT11 sensor");
        Serial.print("Error code = ");
        Serial.println(DHT11state);
    }
#endif
    return DHT11state;
}

/** computeHeatIndex()
 *  @brief Public method that compute the heat index using the values of temperature and humidity
 *         read from the DHT11 sensor
 *  @param tempUnit : temperature unit of measure (by default in Fareneith)
 *  @return the heat index
 *
 *  INFO ABOUT HEAT INDEX (from Wikipedia) :
 *  The heat index (HI) or humiture or humidex is an index that combines air temperature and 
 *  relative humidity, to determine the human-perceived equivalent temperature, 
 *  as how hot it would feel if the humidity were some other value in the shade.
 *  The heat index can be computed using the following equation :
 *  HI = c1 + c2*T + c3*R + c4*T*R + c5*(T^2) + c6*(R^2) + c7*(T^2)*R + c8*T*(R^2) + c9*(T^2)*(R^2)
 *  where :
 *  HI = heat index in degrees Fareneith
 *  T = temperature in degrees Fareneith
 *  R = relative humidity (percentage value between 0 and 100)
 *  c1 = -42.379, c2 = 2.04901523, c3 = 10.14333127, c4 = -0.22475541, c5 = -0.00663783
 *  c6 = -0.05481717, c7 = 0.00122874, c8 = 0.00085282, c9 = -0.00000199
 *
 *  Code based on Adafruit DHT library
 *	
 */
float AccessoryShield::computeHeatIndex(TemperatureUnit tempUnit) {
    // Using both Rothfusz and Steadman's equations
    // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
    float hi;

    delay(1000);
    int8_t DHT11state = getDHT11Data();
    if(DHT11state == DHT11_DATA_READ) {
        hi = 0.5 * (tempF + 61.0 + ((tempF - 68.0) * 1.2) + (relHumidity * 0.094));

        if (hi > 79) {
            hi = -42.379 +
                   2.04901523 * tempF +
                  10.14333127 * relHumidity +
				  -0.22475541 * tempF*relHumidity +
                  -0.00683783 * pow(tempF, 2) +
                  -0.05481717 * pow(relHumidity, 2) +
                   0.00122874 * pow(tempF, 2) * relHumidity +
                   0.00085282 * tempF*pow(relHumidity, 2) +
                  -0.00000199 * pow(tempF, 2) * pow(relHumidity, 2);

                if((relHumidity < 13) && (tempF >= 80.0) && (tempF <= 112.0))
                    hi -= ((13.0 - relHumidity) * 0.25) * sqrt((17.0 - abs(tempF - 95.0)) * 0.05882);

                else if((relHumidity > 85.0) && (tempF >= 80.0) && (tempF <= 87.0))
                    hi += ((relHumidity - 85.0) * 0.1) * ((87.0 - tempF) * 0.2);
        }

        switch(tempUnit) {
        case DHT11_TEMP_CELSIUS:
            return convertTempFtoC(hi);
            break;
        case DHT11_TEMP_FARENEITH:
            return hi;
            break;
        case DHT11_TEMP_KELVIN:
            return convertTempFtoK(hi);
			break;
        default :
            return hi;
        }
    }
    else {
#if defined(SERIAL_DEBUG)
        Serial.println("Warning : Impossible to get temperature and humidity from DHT11 sensor");
        Serial.print("Error code = ");
        Serial.println(DHT11state);
#endif
        return NAN;
    }
}

AccessoryShield accessoryShield;
