/*
  DHT11_data_debug

  Simple sketch that show how to use the DHT11 of the DFRobot
  Accessory Shield to get environmental data and debug info

  created 5 Sept 2016
  by Biagio Montaruli
  
  This example code is in the public domain.
 */
#include <AccessoryShield.h>

float humidity, tempC;

void setup() {
  // initialize Serial comunication
  Serial.begin(9600);
  // wait for the serial port to connect. Needed for Native USB only
  while(!Serial) ;
  // initialize the Accessory Shield library
  accessoryShield.begin();

  // get environmental data from the DHT11 temperature and hunidity sensor
  // using the method getEnvironmentalData()
  // this method returns a value that represents the state of DHT11 sesnor
  // and receives three parameters :
  // 1) a reference to the variable used to store the humidity
  // 2) a reference to the variable used to store the temperature
  // 3) the temperature unit of measure : CELSIUS, FARENEITH or KELVIN;
  //    by default it uses CELSIUS
  int dataState = accessoryShield.getEnvironmentalData(humidity, tempC);
  if(dataState == DHT11_DATA_READ) {
    Serial.println("Successfully read environmental data from DHT11 sensor");
    Serial.print("Temperature in degrees Celsius : ");
    Serial.println(tempC);
    Serial.print("Relative humidity : ");
    Serial.println(humidity); 
}

void loop() {
  
}
