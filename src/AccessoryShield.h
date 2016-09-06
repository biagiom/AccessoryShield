/*
 ####################################################################################
 #  Accessory Shield Library for Arduino/Genuino boards				                #
 #  Copyright (C) 2016 Biagio Montaruli												#
 #  Oled library based on Adafruit_SSD1306 written by Limor Fried/Ladyada			#
 ####################################################################################
 #																					#
 #  AccessoryShield.h	                                                            #
 #  This file is part of Accessory Shield Library.                                  #
 #                                                                                  #
 #  This library is free software: you can redistribute it and/or modify        	#
 #  it under the terms of the GNU Lesser General Public License as published by     #
 #  the Free Software Foundation, either version 3 of the License, or               #
 #  (at your option) any later version.                                             #
 #                                                                                  #
 #  BlackLib library is distributed in the hope that it will be useful,             #
 #  but WITHOUT ANY WARRANTY; without even the implied warranty of                  #
 #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                   #
 #  GNU Lesser General Public License for more details.                             #
 #                                                                                  #
 #  You should have received a copy of the GNU Lesser General Public License        #
 #  along with this program.  If not, see <http://www.gnu.org/licenses/>.           #
 #                                                                                  #
 ####################################################################################
 */

#ifndef ACCESSORYSHIELD_H
#define ACCESSORYSHIELD_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

/* The Accessory Shield uses digital pin 2 and 3 for DHT11 sensor
   and for blue led but on Arduino Leonardo board these pins are also
   used as SCL and SDA pins; so they can't be used to control both the
   OLED display and the DHT11 sensor                                   */
#if !defined(__AVR_ATmega32U4__)
#include "Oled.h"
#endif

/* Uncomment the following line to enable serial debug */
// #define SERIAL_DEBUG

typedef enum {
	CELSIUS = 1,
	FARENEITH = 2,
	KELVIN = 3
} TemperatureUnit;

enum LedPins {
	RED_LED = 9,
	GREEN_LED = 10,
	BLUE_LED = 3
};

enum LedState {
	LED_ON = 255,
	LED_OFF = 0
};

typedef enum {
	JOYSTICK_NONE = 0,
	JOYSTICK_UP = 1,
	JOYSTICK_DOWN = 2,
	JOYSTICK_LEFT = 3,
	JOYSTICK_RIGHT = 4,
	JOYSTICK_PUSH = 5,
	JOYSTICK_NONE_OR_DOWN = 6
} JoystickMode;

enum DHT11State {
	DHT11_CHECKSUM_ERROR = -2,
	DHT11_GETTING_DATA_ERROR = -1,
	DHT11_TIMEOUT_ERROR = 0,
	DHT11_ACK_RECEIVED = 1,
	DHT11_DATA_READ = 2
};

 
#if defined(__AVR_ATmega32U4__)
class AccessoryShield {
#else
class AccessoryShield : public Oled {
#endif

private :
	uint8_t RGBstate[3];
	enum leds {
		r = 0,
		g = 1,
		b = 2,
		numLeds = 3
	};
	
	static const uint8_t potPin =  A1;
	uint16_t potValue;
	
	static const uint8_t buzzerPin = 8;
	bool buzzerState;
	
	static const uint8_t joystickPin = A0;
	uint16_t joystickValue;
	JoystickMode joystickState;
	bool joystickMoved;
	static const char *joystickModes[6];
	
	static const uint8_t DHT11pin = 2;
	uint8_t DHT11data[5];
	float tempC;
	float tempK;
	float tempF;
	float relHumidity;
	
	static const uint8_t relayPin = 11;
	bool relayState;
	
	bool checkPulse(bool state) const;
	int8_t getDHT11Data(void);
	
public :
	AccessoryShield(void);
	void begin(void);
	void end(void);
	
	uint16_t readPot(void);
	
	void setRGB(uint8_t redState = LED_OFF, uint8_t greenState = LED_OFF, uint8_t blueState = LED_OFF);
	uint8_t getRGBstate(uint8_t led);
	void redON(void);
	void redOFF(void);
	void greenON(void);
	void greenOFF(void);
	void blueON(void);
	void blueOFF(void);
	
	void activateBuzzer(void);
	void disableBuzzer(void);
	void playBuzzer(long freq, long delayTime);
	bool getBuzzerState(void) const;
	
	void activateRelay(void);
	void disableRelay(void);
	bool getRelayState(void) const;
	
	JoystickMode getJoystickValue(void);
	
	float getTemperature(TemperatureUnit tempUnit = CELSIUS);
	float getHumidity(void);
	int8_t getEnvironmentalData(float &hum, float &temp, TemperatureUnit tUnit = CELSIUS);
	float convertTempCtoF(float tC);
	float convertTempCtoK(float tC);
	float convertTempFtoC(float tF);
	float convertTempFtoK(float tF);
	float convertTempKtoC(float tK);
	float convertTempKtoF(float tK);
	float computeHeatIndex(TemperatureUnit tempUnit = FARENEITH);
	
};

extern AccessoryShield accessoryShield;

#endif