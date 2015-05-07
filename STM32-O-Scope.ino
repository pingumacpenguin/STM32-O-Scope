/*.
(c) Andrew Hull - 2015

STM32-O-Scope - released under the GNU GENERAL PUBLIC LICENSE Version 2, June 1991

Adafruit Libraries released under their specific licenses Copyright (c) 2013 Adafruit Industries.  All rights reserved.

  Bill of materials.

  eBay links are for reference only, other suppliers may be better, cheaper or more reliable.. or any combination of those three.
  ... pays yer money, you takes yer chance as they say.

  LCD TFT 2.2":  http://www.ebay.com/itm/2-2-inch-2-2-SPI-TFT-LCD-Display-module-240x320-ILI9341-51-AVR-STM32-ARM-PIC-/200939222521    Approx $5.25
   or
  LCD TFT w. Touch Screen http://www.ebay.com/itm/LCD-Touch-Panel-240x320-2-4-SPI-TFT-Serial-Port-Module-With-PBC-ILI9341-5V-3-3V-/291346921118?pt=LH_DefaultDomain_0&hash=item43d5a1369e Approx $6.60
   plus
  STM32F103C8T6 http://www.ebay.com/itm/STM32F103C8T6-ARM-STM32-Minimum-System-Development-Board-Module-for-Arduino-/271845944961?pt=LH_DefaultDomain_0&hash=item3f4b47ee81 Approx $4.59
   plus
  DuPont Wire http://www.ebay.com/itm/40pcs-10cm-1p-1p-female-to-Female-jumper-wire-Dupont-cable-/121247597807?pt=LH_DefaultDomain_0&hash=item1c3aeb84ef Approx $0.99

  Total cost Around $10-$15 or 3 x Cafe Latte Ventis in New York.. *other more accurate international fiscal standards are available.

*/


#include "Adafruit_ILI9341_STM.h"
#include "Adafruit_GFX_AS.h"

#include <SPI.h>
#include <SerialCommand.h>

/* For reference on STM32F103CXXX

variants/generic_stm32f103c/board/board.h:#define BOARD_NR_SPI              2
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_NSS_PIN        PA4
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_MOSI_PIN       PA7
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_MISO_PIN       PA6
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_SCK_PIN        PA5

variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_NSS_PIN        PB12
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_MOSI_PIN       PB15
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_MISO_PIN       PB14
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_SCK_PIN        PB13

*/

// Additional  display specific signals (i.e. non SPI) for STM32F103C8T6 (Wire colour)
#define TFT_DC        PA0      //   (Green) 
#define TFT_CS        PA1      //   (Orange) 
#define TFT_RST       PA2      //   (Yellow)

// Hardware SPI1 on the STM32F103C8T6 *ALSO* needs to be connected and pins are as follows.
//
// SPI1_NSS  (PA4) (LQFP44 pin 14)    (n.c.)
// SPI1_SCK  (PA5) (LQFP44 pin 15)    (Brown)
// SPI1_MOSO (PA6) (LQFP48 pin 16)    (White)
// SPI1_MOSI (PA7) (LQFP48 pin 17)    (Grey)
//

#define TFT_LED        PA3     // Backlight 
#define TEST_WAVE_PIN       PB1     // PWM 500 Hz 

#define PORTRAIT 0
#define LANDSCAPE 1

// Create the lcd object
Adafruit_ILI9341_STM TFT = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RST); // Using hardware SPI

// LED - blinks on trigger events - leave this undefined if your board has no controllable LED
#define BOARD_LED PC13

// Display colours
#define BEAM1_COLOUR ILI9341_GREEN
#define GRATICULE_COLOUR ILI9341_RED
#define BEAM_OFF_COLOUR ILI9341_BLACK
#define CURSOR_COLOUR ILI9341_GREEN

// Analog input
const int8_t analogInPin = PB0;   // Analog input pin: any of LQFP44 pins (PORT_PIN), 10 (PA0), 11 (PA1), 12 (PA2), 13 (PA3), 14 (PA4), 15 (PA5), 16 (PA6), 17 (PA7), 18 (PB0), 19  (PB1)
float samplingTime = 0;

// Samples - depends on available RAM 6K is about the limit on an STM32F103C8T6
// Bear in mind that the ILI9341 display is only able to display 340 pixels, but we can output far more to the serial port.
# define maxSamples 6000
uint16_t startSample = 0;
uint16_t endSample = maxSamples ;
// Variables for the beam position
uint16_t signalX ;
uint16_t signalY ;
uint16_t signalY1;

unsigned long sweepDelayFactor = 1;

// Screen dimensions
int16_t myWidth ;
int16_t myHeight ;
int16_t xZoomFactor = 1;

bool notTriggered ;
int16_t triggerSensitivity;
int16_t retriggerDelay = 1000;

bool onHold = false;
bool serialOutput = false;
// Array for the ADC data
uint16_t dataPoints[maxSamples];

//Array for trigger points
uint16_t triggerPoints[2];

// Create Serial Command Object.
SerialCommand sCmd;

USBSerial serial_debug;



void setup()
{
  serial_debug.begin();

  // BOARD_LED blinks on triggering
#if defined BOARD_LED
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, HIGH);
#endif

  //
  // Serial control setup
  //Serial.begin(2000000);       // Max baudrade depends on port characteristics.

  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("s",   toggleSerial);       // Turns serial sample output on/off
  sCmd.addCommand("h",   toggleHold);         // Turns triggering on/off
  sCmd.addCommand("t",   decreaseTimebase);      // decrease Timebase by 10x
  sCmd.addCommand("T",   increaseTimebase);      // increase Timebase by 10x
  sCmd.addCommand("z",   decreaseZoomFactor);   // decrease Zoom
  sCmd.addCommand("Z",   increaseZoomFactor);   // increase Zoom
  sCmd.addCommand("r",   scrollRight);          // start onscreen trace further right
  sCmd.addCommand("l",   scrollLeft);           // start onscreen trae further left
  /*
  sCmd.addCommand("UPLOAD", uploadSamples);      // Sends the most recent samples
  sCmd.addCommand("TB+",    timeBasePlus);       // Sets Timebase next value
  sCmd.addCommand("TB-",    timeBaseMinus);      // Sets Timebase previous 2
  sCmd.addCommand("TB1",    timeBase1);          // Sets Timebase value 0 (fastest)
  sCmd.addCommand("TB10,    timeBase10);         // Sets Timebase value 10
  sCmd.addCommand("TB100",    timeBase100);      // Sets Timebase value 100
  //
  */
  sCmd.setDefaultHandler(unrecognized);          // Handler for command that isn't matched  (says "Unknown")


  // Backlight, use with caution, depending on your display, you may exceed the max current per pin if you use this method.
  // A safer option would be to add a suitable transistor capable of sinking or sourcing 100mA (the ILI9341 backlight on my display is quouted as drawing 80mA at full brightness)
  // Alternatively, connect the backlight to 3v3 for an always on, bright display.
  pinMode(TFT_LED, OUTPUT);
  analogWrite(TFT_LED, 127);

  // Square wave 3.3V (STM32 supply voltage) at approx 490  Hz
  // "The Arduino has a fixed PWM frequency of 490Hz" - and it appears that this is also true of the STM32F103 using the current STM32F03 libraries as per
  // STM32, Maple and Maple mini port to IDE 1.5.x - http://forum.arduino.cc/index.php?topic=265904.2520

  pinMode(TEST_WAVE_PIN, OUTPUT);
  analogWrite(TEST_WAVE_PIN, 127);

  // Set up our sensor pin(s)
  pinMode(analogInPin, INPUT_ANALOG);

  TFT.begin();
  // initialize the display
  TFT.setRotation(LANDSCAPE);
  clearTFT();
  TFT.setTextSize(2);                           // Small 26 char / line
  TFT.setTextColor(CURSOR_COLOUR, BEAM_OFF_COLOUR) ;
  TFT.setCursor(0, 80);
  TFT.print(" STM-O-Scope by Andy Hull") ;
  TFT.setCursor(0, 100);
  TFT.print("      Inspired by");
  TFT.setCursor(0, 120);
  TFT.print("      Ray Burnette.");
  TFT.setCursor(0, 140);
  TFT.print(" CH1 Probe STM32F Pin [");
  TFT.print(analogInPin);
  TFT.print("]");
  TFT.setRotation(PORTRAIT);
  myHeight   = TFT.width() ;
  myWidth  = TFT.height();
  //xZoomFactor = maxSamples / myWidth;
  graticule();
  delay(5000) ;
  clearTFT();

  notTriggered = true;
  triggerSensitivity = 8 ;
  graticule();
}

void loop()
{
  //serial_debug.println("blah");

  sCmd.readSerial();     // Process serial commands

  if ( !onHold )
  {
    // Wait for trigger
    trigger();
    blinkLED();
    //Blank  out previous plot
    TFTSamples(BEAM_OFF_COLOUR);
    showLabels();

    // Show the Graticule and reset the trigger
    graticule();
    notTriggered = true;

    // Take our samples
    samplingTime = micros();
    takeSamples();
    samplingTime = (micros() - samplingTime);

    // Display the Labels ( uS/Div, Volts/Div etc).
    showLabels();

    //Display the samples
    TFTSamples(BEAM1_COLOUR);
    if (serialOutput)
    {
      serialSamples();
    }
  }
  // Wait before allowing a re-trigger
  delay(retriggerDelay);
  // DEBUG: increment the sweepDelayFactor slowly to show the effect.
  // sweepDelayFactor ++;
}

void graticule()
{
  TFT.drawRect(0, 0, myHeight, myWidth, GRATICULE_COLOUR);
  // Dot grid - ten distinct divisions in both X and Y axis.
  for (uint16_t TicksX = 1; TicksX < 11; TicksX++)
  {
    for (uint16_t TicksY = 1; TicksY < 11; TicksY++)
    {
      TFT.drawPixel(  TicksX * (myHeight / 10), TicksY * (myWidth / 10), GRATICULE_COLOUR);
    }
  }
  // Horizontal and Vertical centre lines
  for (uint16_t TicksX = 0; TicksX < myWidth; TicksX += 10 )
  {
    TFT.drawLine(  (myHeight / 2) - 2 , TicksX, (myHeight / 2) + 2, TicksX, GRATICULE_COLOUR);
  }
  for (uint16_t TicksY = 0; TicksY < myHeight; TicksY += 10 )
  {
    TFT.drawLine( TicksY,  (myWidth / 2) - 2 , TicksY, (myWidth / 2) + 2, GRATICULE_COLOUR);
  }

}

// Crude triggering on positive or negative change from previous to current sample.
void trigger()
{
  triggerPoints[0] = map(analogRead(analogInPin),  0,  4095,  myHeight - 1,  1   ) ;;
  triggerPoints[1] = triggerPoints[0];
  while (notTriggered) {
    triggerPoints[1] = map(analogRead(analogInPin),  0,  4095,  myHeight - 1,  1   ) ;
    if (((triggerPoints[1] - triggerPoints[0] ) > triggerSensitivity) or ((triggerPoints[0] - triggerPoints[1] ) > triggerSensitivity))
    {
      notTriggered = false;
    }
    triggerPoints[0] = triggerPoints[1];
  }
}

void clearTFT()
{
  TFT.fillScreen(BEAM_OFF_COLOUR);                // Blank the display
}

void blinkLED()
{
#if defined BOARD_LED
  digitalWrite(BOARD_LED, LOW);
  delay(10);
  digitalWrite(BOARD_LED, HIGH);
#endif
}

// Grab the samples from the ADC as fast as Arduinoland will let us
// Theoretically the ADC can do 2Ms/S but this would require some optimisation.
void takeSamples ()
{
  for (uint16_t j = 0; j <= maxSamples - 1 ; j++ )
  {
    dataPoints[j] = analogRead(analogInPin);

    // Add NOP delay for reasonably accurate per interval sampling
    // on my test STM32F103CXXX board with no optimisation analogRead can hit minimum <7uS per sample
    // the STM data sheet claims up to 1 mega samples per second for single mode ADC, so we
    // are in the right ballpark here.

    // TODO: Tighten up this loop or better still use DMA and/or dual conversion to get up to 2MS/s i.e. 0.5uS per sample and sub-microsecond accuracy.

    // sweepDelay adds delay factor with a sub uS resolution
    //
    sweepDelay(sweepDelayFactor);
    //delayMicroseconds(13);
  }
}

// TODO: Add a faster samples -> dot mode as well as the current line mode
void TFTSamples (uint16_t beamColour)
{
  signalX = 0;
  // Display the samples scaled to fit the display, full scale fits between the graticules.
  // TODO: Make points 0 and 4096 off the scale i.e. not plotted
  for (uint16_t j = startSample; j <= endSample - xZoomFactor ; j += xZoomFactor )
  {
    signalY =  ((myHeight * dataPoints[j]) / 4096);
    signalY1 = ((myHeight * dataPoints[j + xZoomFactor ]) / 4096);
    TFT.drawLine (  signalY * 99 / 100 + 1, signalX, signalY1 * 99 / 100 + 1 , signalX + 1, beamColour) ;
    signalX += 1;
  }
}

// Run a bunch of NOOPs to trim the inter ADC conversion gap
void sweepDelay(unsigned long sweepDelayFactor) {
  volatile unsigned long i = 0;
  for (i = 0; i < sweepDelayFactor; i++) {
    __asm__ __volatile__ ("nop");
  }
}

void showLabels()
{
  TFT.setRotation(LANDSCAPE);
  TFT.setTextSize(2);
  TFT.setCursor(10, 190);
  TFT.print("Y=");
  TFT.print((10 * samplingTime * xZoomFactor) / maxSamples);
  TFT.print(" uS/Div");
  TFT.setCursor(10, 210);
  TFT.print("X=0.33v/Div");
  TFT.setRotation(PORTRAIT);
}

void serialSamples ()
{
  // Send *all* of the samples to the serial port.
  for (uint16_t j = 1; j <= maxSamples - 1 ; j++ )
  {
    //signalX = j ;
    //signalY =  ((myHeight * dataPoints[j]) / 4096);
    //signalY1 = ((myHeight * dataPoints[j + 1 ]) / 4096);
    //TFT.drawLine (  signalY * 99 / 100 + 1, signalX, signalY1 * 99 / 100 + 1 , signalX + 1, beamColour) ;
    //serial_debug.print("\");
    serial_debug.print(j);
    serial_debug.print(",");
    //serial_debug.print("\"");
    serial_debug.print(dataPoints[j]);
    serial_debug.print("\n");

  }
  serial_debug.print("\n");
}

void toggleHold() {
  onHold = !onHold ;
  serial_debug.println("Toggle Hold");
}

void toggleSerial() {
  serialOutput = !serialOutput ;
  serial_debug.println("Toggle Serial");
  serialSamples();
}

void unrecognized(const char *command) {
  Serial.println("Unknown Command.");
}

void decreaseTimebase() {
  clearTrace();
  sweepDelayFactor =  sweepDelayFactor / 2 ;
  if (sweepDelayFactor < 1 ) {

    serial_debug.print("Timebase=");
    sweepDelayFactor = 1;
  }
  showTrace();
  serial_debug.println(sweepDelayFactor);
}

void increaseTimebase() {
  clearTrace();
  serial_debug.print("Timebase=");
  sweepDelayFactor = 2 * sweepDelayFactor ;
  showTrace();
  serial_debug.println(sweepDelayFactor);
}

void increaseZoomFactor() {
  clearTrace();
  if ( xZoomFactor < 18) {
    xZoomFactor += 1;
  }
  showTrace();
  serial_debug.print("Zoom=");
  serial_debug.println(xZoomFactor);

}

void decreaseZoomFactor() {
  clearTrace();
  if (xZoomFactor > 1) {
    xZoomFactor -= 1;
  }
  showTrace();
  Serial.print("Zoom=");
  Serial.println(xZoomFactor);
  //clearTFT();
}

void clearTrace() {
  TFTSamples(BEAM_OFF_COLOUR);
  graticule();
}

void showTrace() {
  showLabels();
  TFTSamples(BEAM1_COLOUR);
}

void scrollRight() {
  clearTrace();
  if (startSample < (endSample - 12)) {
    startSample += 10;
  }
  showTrace();
  Serial.print("startSample=");
  Serial.println(startSample);


}

void scrollLeft() {
  clearTrace();
  if (startSample > (12)) {
    startSample -= 10;
    showTrace();
  }
  Serial.print("startSample=");
  Serial.println(startSample);


}

