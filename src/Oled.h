/***************************************************************************
This is a library for Monochrome OLED display of DFRobot Accessory Shield
The OLED display uses the SSD1306 driver thought the I2C interface.

The OLED library is based on Adafruit_SSD1306 library which has been
modified in order to support the OLED display of Accessory Shield.
Many thanks to the library writer Limor Fried/Ladyada and to Adafruit
team which provide many and great open source code

*****************************************************************************/
#ifndef _OLED_H_
#define _OLED_H_

#include <Wire.h>

#if ARDUINO >= 100
    #include "Arduino.h"
    #define WIRE_WRITE Wire.write
#else
    #include "WProgram.h"
    #define WIRE_WRITE Wire.send
#endif

#include <Adafruit_GFX.h>

#define BLACK   0
#define WHITE   1
#define INVERSE 2

/*=======================================================================

SSD1306 Displays
--------------------------------------------------------------------
The SSDF1306 driver is used in multiple displays (128x64, 128x32, etc.).
The DFRobot Accessory Shield uses a 128x64 pixel OLED display
-----------------------------------------------------------------------*/
#define SSD1306_LCDWIDTH    128
#define SSD1306_LCDHEIGHT   64

/*======================================================================*/

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON    0xA5
#define SSD1306_NORMALDISPLAY   0xA6
#define SSD1306_INVERTDISPLAY   0xA7
#define SSD1306_DISPLAYOFF  0xAE
#define SSD1306_DISPLAYON   0xAF

#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETCOMPINS  0xDA

#define SSD1306_SETVCOMDETECT   0xDB

#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE    0xD9

#define SSD1306_SETMULTIPLEX    0xA8

#define SSD1306_SETLOWCOLUMN    0x00
#define SSD1306_SETHIGHCOLUMN   0x10

#define SSD1306_SETSTARTLINE    0x40

#define SSD1306_MEMORYMODE  0x20
#define SSD1306_COLUMNADDR  0x21
#define SSD1306_PAGEADDR    0x22

#define SSD1306_COMSCANINC  0xC0
#define SSD1306_COMSCANDEC  0xC8

#define SSD1306_SEGREMAP    0xA0

#define SSD1306_CHARGEPUMP  0x8D

//#define SSD1306_EXTERNALVCC   0x1
//#define SSD1306_SWITCHCAPVCC  0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL   0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA    0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL  0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL    0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A

class Oled : public Adafruit_GFX {

public:
    void oledCommand(uint8_t c);

    void clearOledDisplay(void);
    void invertOledDisplay(uint8_t i);
    void oledPaint();

    void startScrollRightOled(uint8_t start, uint8_t stop);
    void startScrollLeftOled(uint8_t start, uint8_t stop);

    void startScrollDiagRightOled(uint8_t start, uint8_t stop);
    void startScrollDiagLeftOled(uint8_t start, uint8_t stop);
    void stopScrollOled(void);

    void dimOled(boolean dim);

    void drawPixel(int16_t x, int16_t y, uint16_t color);

    virtual void drawFastVLineOled(int16_t x, int16_t y, int16_t h, uint16_t color);
    virtual void drawFastHLineOled(int16_t x, int16_t y, int16_t w, uint16_t color);

protected:
    Oled(void);
    void oledBegin(void);

private:
    static const uint8_t i2cAddr = 0x3C;  // 011110+SA0+RW - 0x3C or 0x3D
    static const uint8_t oledReset = 6;
    static const uint8_t oledDC = 7;

    inline void drawFastVLineInternal(int16_t x, int16_t y, int16_t h, uint16_t color) __attribute__((always_inline));
    inline void drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) __attribute__((always_inline));
};

#endif /* _OLED_H_ */
