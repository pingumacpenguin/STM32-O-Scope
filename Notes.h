/* http://forum.arduino.cc/index.php?topic=265904.msg2077121#msg2077121
    Forum user: madias, Vienna, AUSTRIA:  "I if modified the Adafruit Ili9341 library, for all 
    people, who didn't apply the SPI patch on thread number #1361. But I'll strongly recommend to 
    apply timschuerewegen modifications. Must have bugfixes! There was no #define for  
    digitalPinToBitMask/digitalPinToPort...  so I copied them from UTFT at the begin of Adafruit_ili9341.cpp.
    (are those digitalPin**** now included in the newest STM32 release?) Then I got a mismatch in the *.h file 
    because of volatile uint32 * VS volatile uint32_t *. I corrected this."
    Using library: adafruit ili9341, hardware SPI
Pin connections:
    #define TFT_DC 12
    #define TFT_CS 13
    #define TFT_RST 14
    the rest of the pins (MISO,MOSI,SCK) are the standard SPI pins of your mini.

Maple Mini results:
    ILI9341 Test!
    Display Power Mode:  0x0
    MADCTL Mode:         0x0
    Pixel Format:        0x0
    Image Format:        0x0
    Self Diagnostic:     0x0
    Benchmark                Time (microseconds)
    Screen fill              1026801
    Text                     74912
    Lines                    702835
    Horiz/Vert Lines         84366
    Rectangles (outline)     54482
    Rectangles (filled)      2132433
    Circles (filled)         344627
    Circles (outline)        306304
    Triangles (outline)      222952
    Triangles (filled)       715611
    Rounded rects (outline)  130140
    Rounded rects (filled)   2331430
    Done!                              */

/*
  ILI9341 TFT GLCD display connections for hardware SPI:
  Signal           Maple Mini           Leonardo      LCD Display    UNO pins
  ===============  ===========          ========      ===========    ========
  #define _sclk         6         //         15       J2 pin 7          13
  #define _miso         5 NC      //         14          pin 9          12
  #define _mosi         4         //         16          pin 6          11
  #define TFT_CS       13         //         10          pin 3          10
  #define TFT_DC       12         //          9          pin 5           9
  #define TFT_RST      14         //          8          pin 4           8
*/

/*
 Color definitions for TFT SPI 2.2" Display
    ILI9341_BLACK   0x0000
    ILI9341_BLUE    0x001F
    ILI9341_RED     0xF800
    ILI9341_GREEN   0x07E0
    ILI9341_CYAN    0x07FF
    ILI9341_MAGENTA 0xF81F
    ILI9341_YELLOW  0xFFE0  
    ILI9341_WHITE   0xFFFF


**************************** ILI9341 320x240 DISPLAY LAYOUT ****************************
0,0
----------------------------------------------------------------------------> 319
| nnnT                                                                 DOW
|              lcd.fillRect( 0,  0, 319, 59, 0);     // blank top
|
|
|<- 059 
|
|              lcd.fillRect( 0, 60, 319, 114, 0);     // blank middle
|                              HH:MM:SS A
|
|
|
|<- 174
|              lcd.fillRect( 0, 175, 319, 64, 0);     // blank lower
|
| MON DD YYYY
|
|<- 239
*/

/*
 https://learn.adafruit.com/downloads/pdf/adafruit-gfx-graphics-library.pdf

 **************************** Adafruit_GFX functions ****************************

>   Drawing pixels (points)
      void drawPixel(uint16_t x, uint16_t y, uint16_t color);
    
>   Drawing lines
      void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
      void drawFastVLine(uint16_t x0, uint16_t y0, uint16_t length, uint16_t color);
      void drawFastHLine(uin86_t x0, uin86_t y0, uint8_t length, uint16_t color);
      
>   Rectangles
      void drawRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color);
      void fillRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color);
    
>   Circles
      void drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
      void fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
    
>   Rounded rectangles
      void drawRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color);
      void fillRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color);
    
>   Triangles
      void drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
      void fillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
    
>   Characters and text
      void drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
      void setCursor(uint16_t x0, uint16_t y0);
      void setTextColor(uint16_t color);
      void setTextColor(uint16_t color, uint16_t backgroundcolor);
      void setTextSize(uint8_t size);
      void setTextWrap(boolean w);
    
>   Bitmaps
      void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
    
>   Clearing or filling the screen
      void fillScreen(uint16_t color);
    
>   Rotating the Display
      void setRotation(uint8_t rotation);  // 0, 1, 2 or 3
    
>   Reference the size of the screen
      uint16_t width();
      uint16_t height();

 */
 
