# Accessory Shield Library for Arduino boards

This library allows an Arduino board to use the actuators, sensors and OLED display of the DFRobot Accessory Shield.

For more info see the [Product page on DFRobot web site](https://www.dfrobot.com/index.php?route=product/product&product_id=1045)
or the [Wiki page](https://www.dfrobot.com/wiki/index.php/Accessory_Shield_for_Arduino_SKU:DFR0270)

To control the OLED display through the I2C interface, the Accessory Shield Library includes a modified version of 
the Adafruit_SSD1306 called OLED since the DFRobot Accessory Shield has a Monochrome OLED display based on SSD1306 driver.
Many thanks to Limor Fried/Ladyada and to Adafruit team for providing and maintaining the great open source code
of Adafruit_SSD1306 library.
In order to use the OLED library you must download and install the Adafruit GFX Graphics library that is used as the
core graphic library to display geometric shapes, text and bitmap images on the OLED display.
You can get it from https://github.com/adafruit/Adafruit-GFX-Library; moreover you can also install it through the
Arduino Library Manager. For more info about the Library Manager and Arduino libraries see : 
https://www.arduino.cc/en/Guide/Libraries


### License

Copyright (c) 2017 Biagio Montaruli.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
