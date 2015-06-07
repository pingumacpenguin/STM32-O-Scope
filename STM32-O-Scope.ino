/*.
(c) Andrew Hull - 2015

STM32-O-Scope - aka "The Pig Scope" or pigScope released under the GNU GENERAL PUBLIC LICENSE Version 2, June 1991

https://github.com/pingumacpenguin/STM32-O-Scope

Adafruit Libraries released under their specific licenses Copyright (c) 2013 Adafruit Industries.  All rights reserved.

*/

#include "Adafruit_ILI9341_STM.h"
#include "Adafruit_GFX_AS.h"

// Be sure to use the latest version of the SPI libraries see stm32duino.com - http://stm32duino.com/viewtopic.php?f=13&t=127
#include <SPI.h>

#define PORTRAIT 0
#define LANDSCAPE 1

// Define the orientation of the touch screen. Further
// information can be found in the UTouch library documentation.

#define TOUCH_SCREEN_AVAILABLE
#define TOUCH_ORIENTATION  LANDSCAPE

#if defined TOUCH_SCREEN_AVAILABLE
// UTouch Library
// http://www.rinkydinkelectronics.com/library.php?id=56
#include <UTouch.h>

#endif
// Initialize touchscreen
// ----------------------
// Set the pins to the correct ones for your STM32F103 board
// -----------------------------------------------------------

//
// STM32F103C8XX Pin numbers - chosen for ease of use on the "Red Pill" and "Blue Pill" board

// Touch Panel Pins
// T_CLK T_CS T_DIN T_DOUT T_IRQ
// PB12 PB13 PB14 PB15 PA8
// Example wire colours Brown,Red,Orange,Yellow,Violet

#if defined TOUCH_SCREEN_AVAILABLE

UTouch  myTouch( PB12, PB13, PB14, PB15, PA8);

#endif

// RTC and NVRam initialisation

#include "RTClock.h"
RTClock rt (RTCSEL_LSE); // initialise
uint32 tt;

// Define the Base address of the RTC  registers (battery backed up CMOS Ram), so we can use them for config of touch screen and other calibration.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.

#define BKP_REG_BASE   (uint32_t *)(0x40006C00 +0x04)

// #define NVRam register names for the touch calibration values.
#define  TOUCH_CALIB_X 0
#define  TOUCH_CALIB_Y 1
#define  TOUCH_CALIB_Z 2

// Time library - https://github.com/PaulStoffregen/Time
#include "Time.h"
#define TZ    "UTC+1"

// End RTC and NVRam initialization

// SeralCommand -> https://github.com/kroimon/Arduino-SerialCommand.git
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
// SPI1_NSS  (PA4) (LQFP48 pin 14)    (n.c.)
// SPI1_SCK  (PA5) (LQFP48 pin 15)    (Brown)
// SPI1_MOSO (PA6) (LQFP48 pin 16)    (White)
// SPI1_MOSI (PA7) (LQFP48 pin 17)    (Grey)
//

#define TFT_LED        PA3     // Backlight 
#define TEST_WAVE_PIN       PB0     // PWM 500 Hz 

// Create the lcd object
Adafruit_ILI9341_STM TFT = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RST); // Using hardware SPI

// LED - blinks on trigger events - leave this undefined if your board has no controllable LED
// define as PC13 on the "Red/Blue Pill" boards and PD2 on the "Yellow Pill R"
#define BOARD_LED PC13

// Display colours
#define BEAM1_COLOUR ILI9341_GREEN
//#define GRATICULE_COLOUR ILI9341_RED
#define GRATICULE_COLOUR 0x07FF
#define BEAM_OFF_COLOUR ILI9341_BLACK
#define CURSOR_COLOUR ILI9341_GREEN

// Analog input
#define ANALOG_MAX_VALUE 4096
const int8_t analogInPin = PB1;   // Analog input pin: any of LQFP44 pins (PORT_PIN), 10 (PA0), 11 (PA1), 12 (PA2), 13 (PA3), 14 (PA4), 15 (PA5), 16 (PA6), 17 (PA7), 18 (PB0), 19  (PB1)
float samplingTime = 0;


// Variables for the beam position
uint16_t signalX ;
uint16_t signalY ;
uint16_t signalY1;
int16_t xZoomFactor = 1;
// yZoomFactor (percentage)
int16_t yZoomFactor = 200;
int16_t yPosition = -150 ;

// Startup with sweep hold off or on
boolean triggerHeld = 0 ;


unsigned long sweepDelayFactor = 1;
unsigned long timeBase = 100;  // Timebase in microseconds

// Screen dimensions
int16_t myWidth ;
int16_t myHeight ;

//Trigger stuff
boolean notTriggered ;

// Sensitivity is the necessary change in AD value which will cause the scope to trigger.
// If VAD=3.3 volts, then 1 unit of sensitivity is around 0.8mV but this assumes no external attenuator. Calibration is needed to match this with the magnitude of the input signal.
int32_t triggerSensitivity = 20;

int16_t retriggerDelay = 10;
int8_t triggerType = 1;

//Array for trigger points
uint16_t triggerPoints[2];


// Serial output of samples - off by default. Toggled from UI/Serial commands.
boolean serialOutput = false;

// Create Serial Command Object.
SerialCommand sCmd;

// Create USB serial port
USBSerial serial_debug;

// Samples - depends on available RAM 6K is about the limit on an STM32F103C8T6
// Bear in mind that the ILI9341 display is only able to display 240x320 pixels, at any time but we can output far more to the serial port, we effectively only show a window on our samples on the TFT.
# define maxSamples 1024*6
uint32_t startSample = 10;
uint32_t endSample = maxSamples ;


// Array for the ADC data
//uint16_t dataPoints[maxSamples];
uint32_t dataPoints32[maxSamples / 2];
uint16_t *dataPoints = (uint16_t *)&dataPoints32;

// End of DMA indication
volatile static bool dma1_ch1_Active;
#define ADC_CR1_FASTINT 0x70000 // Fast interleave mode DUAL MODE bits 19-16



void setup()
{

  // BOARD_LED blinks on triggering assuming you have an LED on your board. If not simply dont't define it at the start of the sketch.
#if defined BOARD_LED
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, HIGH);
  delay(1000);
  digitalWrite(BOARD_LED, LOW);
  delay(1000);
#endif

  serial_debug.begin();
  adc_calibrate(ADC1);
  adc_calibrate(ADC2);
  setADCs (); //Setup ADC peripherals for interleaved continuous mode.
  /*
    // BOARD_LED blinks on triggering assuming you have an LED on your board. If not simply dont't define it at the start of the sketch.
  #if defined BOARD_LED
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, HIGH);
  #endif
  */
  //
  // Serial command setup
  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("timestamp",   setCurrentTime);          // Set the current time based on a unix timestamp
  sCmd.addCommand("date",        serialCurrentTime);       // Show the current time from the RTC

#if defined TOUCH_SCREEN_AVAILABLE
  sCmd.addCommand("touchcalibrate", touchCalibrate);       // Calibrate Touch Panel
#endif

  sCmd.addCommand("s",   toggleSerial);                    // Turns serial sample output on/off
  sCmd.addCommand("h",   toggleHold);                      // Turns triggering on/off
  sCmd.addCommand("t",   decreaseTimebase);                // decrease Timebase by 10x
  sCmd.addCommand("T",   increaseTimebase);                // increase Timebase by 10x
  sCmd.addCommand("z",   decreaseZoomFactor);              // decrease Zoom
  sCmd.addCommand("Z",   increaseZoomFactor);              // increase Zoom
  sCmd.addCommand("r",   scrollRight);                     // start onscreen trace further right
  sCmd.addCommand("l",   scrollLeft);                      // start onscreen trae further left
  sCmd.addCommand("e",   incEdgeType);                     // increment the trigger edge type 0 1 2 0 1 2 etc
  sCmd.addCommand("y",   decreaseYposition);               // move trace Down
  sCmd.addCommand("Y",   increaseYposition);               // move trace Down
  sCmd.addCommand("P",   toggleTestPulseOn);               // Toggle the test pulse pin from high impedence input to square wave output.
  sCmd.addCommand("p",   toggleTestPulseOff);              // Toggle the Test pin from square wave test to high impedence input.

  sCmd.setDefaultHandler(unrecognized);                    // Handler for command that isn't matched  (says "Unknown")
  sCmd.clearBuffer();

  // Backlight, use with caution, depending on your display, you may exceed the max current per pin if you use this method.
  // A safer option would be to add a suitable transistor capable of sinking or sourcing 100mA (the ILI9341 backlight on my display is quoted as drawing 80mA at full brightness)
  // Alternatively, connect the backlight to 3v3 for an always on, bright display.
  //pinMode(TFT_LED, OUTPUT);
  //analogWrite(TFT_LED, 127);


  // Setup Touch Screen
  // http://www.rinkydinkelectronics.com/library.php?id=56
#if defined TOUCH_SCREEN_AVAILABLE
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_HI);
#endif


  // The test pulse is a square wave of approx 3.3V (i.e. the STM32 supply voltage) at approx 490  Hz
  // "The Arduino has a fixed PWM frequency of 490Hz" - and it appears that this is also true of the STM32F103 using the current STM32F03 libraries as per
  // STM32, Maple and Maple mini port to IDE 1.5.x - http://forum.arduino.cc/index.php?topic=265904.2520
  // therefore if we want a precise test frequency we can't just use the default uncooked 50% duty cycle PWM output.
  timer_set_period(Timer3, 1000);
  toggleTestPulseOn();

  // Set up our sensor pin(s)
  pinMode(analogInPin, INPUT_ANALOG);

  TFT.begin();
  // initialize the display
  clearTFT();
  TFT.setRotation(PORTRAIT);
  myHeight   = TFT.width() ;
  myWidth  = TFT.height();
  TFT.setTextColor(CURSOR_COLOUR, BEAM_OFF_COLOUR) ;
#if defined TOUCH_SCREEN_AVAILABLE
  touchCalibrate();
#endif

  TFT.setRotation(LANDSCAPE);
  clearTFT();
  showCredits(); // Honourable mentions ;¬)
  showGraticule();
  delay(5000) ;
  clearTFT();
  notTriggered = true;
  showGraticule();
  showLabels();
}

void loop()
{

#if defined TOUCH_SCREEN_AVAILABLE
  readTouch();
#endif

  sCmd.readSerial();     // Process serial commands
  if ( !triggerHeld  )
  {
    // Wait for trigger
    trigger();
    showGraticule();
    if ( !notTriggered )
    {
      blinkLED();
      //Blank  out previous plot
      TFTSamples(BEAM_OFF_COLOUR);
      showLabels();

      // Show the showGraticule
      showGraticule();
      //notTriggered = true;

      // Take our samples
      takeSamples();

      // Display the Labels ( uS/Div, Volts/Div etc).
      showLabels();

      //Display the samples
      TFTSamples(BEAM1_COLOUR);
      /*
      if (serialOutput)
      {
        serialSamples();
      }
      */
    }
    showTime();
  }
  // Wait before allowing a re-trigger
  delay(retriggerDelay);
  // DEBUG: increment the sweepDelayFactor slowly to show the effect.
  // sweepDelayFactor ++;
}

void showGraticule()
{
  TFT.drawRect(0, 0, myHeight, myWidth, GRATICULE_COLOUR);
  // Dot grid - ten distinct divisions (9 dots) in both X and Y axis.
  for (uint16_t TicksX = 1; TicksX < 10; TicksX++)
  {
    for (uint16_t TicksY = 1; TicksY < 10; TicksY++)
    {
      TFT.drawPixel(  TicksX * (myHeight / 10), TicksY * (myWidth / 10), GRATICULE_COLOUR);
    }
  }
  // Horizontal and Vertical centre lines 5 ticks per grid square with a longer tick in line with our dots
  for (uint16_t TicksX = 0; TicksX < myWidth; TicksX += (myHeight / 50))
  {
    if (TicksX % (myWidth / 10) > 0 )
    {
      TFT.drawFastHLine(  (myHeight / 2) - 2 , TicksX, 5, GRATICULE_COLOUR);
    }
    else
    {
      TFT.drawFastHLine(  (myHeight / 2) - 6 , TicksX, 11, GRATICULE_COLOUR);
    }

  }
  for (uint16_t TicksY = 0; TicksY < myHeight; TicksY += (myHeight / 50) )
  {
    if (TicksY % (myHeight / 10) > 0 )
    {
      TFT.drawFastVLine( TicksY,  (myWidth / 2) - 2 , 5, GRATICULE_COLOUR);
    }
    else
    {
      TFT.drawFastVLine( TicksY,  (myWidth / 2) - 5 , 11, GRATICULE_COLOUR);
    }
  }
}

void setADCs ()
{
  //  const adc_dev *dev = PIN_MAP[analogInPin].adc_device;
  int pinMapADCin = PIN_MAP[analogInPin].adc_channel;
  adc_set_sample_rate(ADC1, ADC_SMPR_13_5);
  adc_set_sample_rate(ADC2, ADC_SMPR_13_5);

  //  adc_reg_map *regs = dev->regs;
  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->SQR3 = pinMapADCin;
  ADC1->regs->CR2 |= ADC_CR2_CONT; // | ADC_CR2_DMA; // Set continuous mode and DMA
  ADC1->regs->CR1 |= ADC_CR1_FASTINT; // Interleaved mode
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;

  ADC2->regs->CR2 |= ADC_CR2_CONT; // ADC 2 continuos
  ADC2->regs->SQR3 = pinMapADCin;
}


// Crude triggering on positive or negative or either change from previous to current sample.
void trigger()
{
  /*
  for (uint16_t j = 0; j <= 1000 ; j++ )
  {
    analogRead(analogInPin);
  }
  */

  notTriggered = true;
  switch (triggerType) {
    case 1:
      triggerNegative() ;
      break;
    case 2:
      triggerPositive() ;
      break;
    default:
      triggerBoth() ;
      break;
  }
}

void triggerBoth()
{
  triggerPoints[0] = analogRead(analogInPin);
  delayMicroseconds(20);
  if (((analogRead(analogInPin) - triggerPoints[0] ) < triggerSensitivity) and ((triggerPoints[0] - analogRead(analogInPin) ) < triggerSensitivity)) {
    notTriggered = false ;
  }
}

void triggerPositive() {
  //triggerPoints[0] = analogRead(analogInPin);
  //delayMicroseconds(20);
  triggerPoints[1] = analogRead(analogInPin);
  if ((triggerPoints[1] - triggerPoints[0] ) > triggerSensitivity) {
    notTriggered = false;
  }
  triggerPoints[0] = analogRead(analogInPin);
}

void triggerNegative() {
  //triggerPoints[0] = analogRead(analogInPin);
  //delayMicroseconds(20);
  triggerPoints[1] = analogRead(analogInPin);
  if ((triggerPoints[0] - triggerPoints[1] ) > triggerSensitivity) {
    notTriggered = false;
  }
  triggerPoints[0] = analogRead(analogInPin);
}

void incEdgeType() {
  triggerType += 1;
  if (triggerType > 2)
  {
    triggerType = 0;
  }
  serial_debug.println(triggerPoints[0]);
  serial_debug.println(triggerPoints[1]);
  serial_debug.println(triggerType);
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

// Grab the samples from the ADC
// Theoretically the ADC can not go any faster than this.
//
// According to specs, when using 72Mhz on the MCU main clock,the fastest ADC capture time is 1.17 uS. As we use 2 ADCs we get double the captures, so .58 uS, which is the times we get with ADC_SMPR_1_5.
// I think we have reached the speed limit of the chip, now all we can do is improve accuracy.
// See; http://stm32duino.com/viewtopic.php?f=19&t=107&p=1202#p1194

void takeSamples ()
{
  // This loop uses dual interleaved mode to get the best performance out of the ADCs
  //

  dma_init(DMA1);
  dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);

  adc_dma_enable(ADC1);
  dma_setup_transfer(DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_32BITS,
                     dataPoints32, DMA_SIZE_32BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));// Receive buffer DMA
  dma_set_num_transfers(DMA1, DMA_CH1, maxSamples / 2);
  dma1_ch1_Active = 1;
  //  regs->CR2 |= ADC_CR2_SWSTART; //moved to setADC
  dma_enable(DMA1, DMA_CH1); // Enable the channel and start the transfer.
  //adc_calibrate(ADC1);
  //adc_calibrate(ADC2);
  samplingTime = micros();
  while (dma1_ch1_Active);
  samplingTime = (micros() - samplingTime);

  dma_disable(DMA1, DMA_CH1); //End of trasfer, disable DMA and Continuous mode.
  // regs->CR2 &= ~ADC_CR2_CONT;
  /*
  for (int16_t j = 0; j < maxSamples - 1  ; j++ )
  {

   uint16_t dataPointsAve =  (dataPoints32[j] + dataPoints[j+1])/2;

    // dataPoints32[j] &=0x0FC00FC0;
  }
  */
}



void TFTSamples (uint16_t beamColour)
{
  signalX = 1;
  while (signalX < myWidth - 2)
  {
    // Scale our samples to fit our screen. Most scopes increase this in steps of 5,10,25,50,100 250,500,1000 etc
    // Pick the nearest suitable samples for each of our myWidth screen resolution points
    signalY =  ((myHeight * dataPoints[signalX * ((endSample - startSample) / (myWidth * timeBase / 100)) + 1]) / ANALOG_MAX_VALUE) * (yZoomFactor / 100) + yPosition;
    signalY1 = ((myHeight * dataPoints[(signalX + 1) * ((endSample - startSample) / (myWidth * timeBase / 100)) + 1]) / ANALOG_MAX_VALUE) * (yZoomFactor / 100) + yPosition ;
    TFT.drawLine (  signalY * 99 / 100 + 1, signalX, signalY1 * 99 / 100 + 1 , signalX + 1, beamColour) ;
    signalX += 1;
  }
}

/*
// Run a bunch of NOOPs to trim the inter ADC conversion gap
void sweepDelay(unsigned long sweepDelayFactor) {
  volatile unsigned long i = 0;
  for (i = 0; i < sweepDelayFactor; i++) {
    __asm__ __volatile__ ("nop");
  }
}
*/

void showLabels()
{
  TFT.setRotation(LANDSCAPE);
  TFT.setTextSize(2);
  TFT.setCursor(10, 190);
  // TFT.print("Y=");
  //TFT.print((samplingTime * xZoomFactor) / maxSamples);
  TFT.print(float (float(samplingTime) / float(maxSamples)));

  TFT.setTextSize(1);
  TFT.print(" uS/Sample ");
  TFT.setTextSize(2);
  TFT.setCursor(10, 210);
  TFT.print("3.0");
  TFT.setTextSize(1);
  TFT.print(" V/Div ");
  TFT.setTextSize(2);
  TFT.print(samplingTime);
  TFT.setTextSize(1);
  TFT.print(" us for ");
  TFT.print(maxSamples);
  TFT.print(" samples ");
  //showTime();
  TFT.setRotation(PORTRAIT);
}

void showTime ()
{
  // Show RTC Time.
  TFT.setTextSize(1);
  TFT.setRotation(LANDSCAPE);
  if (rt.getTime() != tt)
  {
    tt = rt.getTime();
    TFT.setCursor(5, 10);
    if (hour(tt) < 10) {
      TFT.print("0");
    }
    TFT.print(hour(tt));
    TFT.print(":");
    if (minute(tt) < 10) {
      TFT.print("0");
    }
    TFT.print(minute(tt));
    TFT.print(":");
    if (second(tt) < 10) {
      TFT.print("0");
    }
    TFT.print(second(tt));
    TFT.print(" ");
    TFT.print(day(tt));
    TFT.print("-");
    TFT.print(month(tt));
    TFT.print("-");
    TFT.print(year(tt));
    TFT.print(" "TZ" ");
    // TFT.print(tt);

  }
  TFT.setRotation(PORTRAIT);
}

void serialSamples ()
{
  // Send *all* of the samples to the serial port.
  serial_debug.println("#Time(uS), ADC Number, value, diff");
  for (int16_t j = 1; j < maxSamples   ; j++ )
  {
    // Time from trigger in milliseconds
    serial_debug.print((samplingTime / (maxSamples))*j);
    serial_debug.print(" ");
    // raw ADC data
    serial_debug.print(j % 2 + 1);
    serial_debug.print(" ");
    serial_debug.print(dataPoints[j] );
    serial_debug.print(" ");
    serial_debug.print(dataPoints[j] - dataPoints[j - 1]);
    serial_debug.print(" ");
    serial_debug.print(dataPoints[j] - ((dataPoints[j] - dataPoints[j - 1]) / 2));
    serial_debug.print("\n");

    // delay(100);


  }
  serial_debug.print("\n");
}

void toggleHold()
{
  triggerHeld ^= 1 ;
  //serial_debug.print("# ");
  //serial_debug.print(triggerHeld);
  if (triggerHeld)
  {
    serial_debug.println("# Toggle Hold on");
  }
  else
  {
    serial_debug.println("# Toggle Hold off");
  }
}

void toggleSerial() {
  serialOutput = !serialOutput ;
  serial_debug.println("# Toggle Serial");
  serialSamples();
}

void unrecognized(const char *command) {
  serial_debug.print("# Unknown Command.[");
  serial_debug.print(command);
  serial_debug.println("]");
}

void decreaseTimebase() {
  clearTrace();
  /*
  sweepDelayFactor =  sweepDelayFactor / 2 ;
  if (sweepDelayFactor < 1 ) {

    serial_debug.print("Timebase=");
    sweepDelayFactor = 1;
  }
  */
  if (timeBase > 100)
  {
    timeBase -= 100;
  }
  showTrace();
  serial_debug.print("# Timebase=");
  serial_debug.println(timeBase);

}

void increaseTimebase() {
  clearTrace();
  serial_debug.print("# Timebase=");
  if (timeBase < 10000)
  {
    timeBase += 100;
  }
  //sweepDelayFactor = 2 * sweepDelayFactor ;
  showTrace();
  serial_debug.print("# Timebase=");
  serial_debug.println(timeBase);
}

void increaseZoomFactor() {
  clearTrace();
  if ( xZoomFactor < 21) {
    xZoomFactor += 1;
  }
  showTrace();
  serial_debug.print("# Zoom=");
  serial_debug.println(xZoomFactor);

}

void decreaseZoomFactor() {
  clearTrace();
  if (xZoomFactor > 1) {
    xZoomFactor -= 1;
  }
  showTrace();
  Serial.print("# Zoom=");
  Serial.println(xZoomFactor);
  //clearTFT();
}

void clearTrace() {
  TFTSamples(BEAM_OFF_COLOUR);
  showGraticule();
}

void showTrace() {
  showLabels();
  TFTSamples(BEAM1_COLOUR);
}

void scrollRight() {
  clearTrace();
  if (startSample < (endSample - 120)) {
    startSample += 100;
  }
  showTrace();
  Serial.print("# startSample=");
  Serial.println(startSample);


}

void scrollLeft() {
  clearTrace();
  if (startSample > (120)) {
    startSample -= 100;
    showTrace();
  }
  Serial.print("# startSample=");
  Serial.println(startSample);

}

void increaseYposition() {

  if (yPosition < myHeight ) {
    clearTrace();
    yPosition ++;
    showTrace();
  }
  Serial.print("# yPosition=");
  Serial.println(yPosition);
}

void decreaseYposition() {

  if (yPosition > -myHeight ) {
    clearTrace();
    yPosition --;
    showTrace();
  }
  Serial.print("# yPosition=");
  Serial.println(yPosition);
}

void atAt() {
  serial_debug.println("# Hello");
}

void toggleTestPulseOn () {
  pinMode(TEST_WAVE_PIN, OUTPUT);
  analogWrite(TEST_WAVE_PIN, 75);
  serial_debug.println("# Test Pulse On.");
}

void toggleTestPulseOff () {
  pinMode(TEST_WAVE_PIN, INPUT);
  serial_debug.println("# Test Pulse Off.");
}

uint16 timer_set_period(HardwareTimer timer, uint32 microseconds) {
  if (!microseconds) {
    timer.setPrescaleFactor(1);
    timer.setOverflow(1);
    return timer.getOverflow();
  }

  uint32 cycles = microseconds * (72000000 / 1000000); // 72 cycles per microsecond

  uint16 ps = (uint16)((cycles >> 16) + 1);
  timer.setPrescaleFactor(ps);
  timer.setOverflow((cycles / ps) - 1 );
  return timer.getOverflow();
}

/**
* @brief Enable DMA requests
* @param dev ADC device on which to enable DMA requests
*/

void adc_dma_enable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 1);
}


/**
* @brief Disable DMA requests
* @param dev ADC device on which to disable DMA requests
*/

void adc_dma_disable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 0);
}

static void DMA1_CH1_Event() {
  dma1_ch1_Active = 0;
}

void setCurrentTime() {
  char *arg;
  arg = sCmd.next();
  String thisArg = arg;
  serial_debug.print("# Time command [");
  serial_debug.print(thisArg.toInt() );
  serial_debug.println("]");
  setTime(thisArg.toInt());
  time_t tt = now();
  rt.setTime(tt);
  serialCurrentTime();
}

void serialCurrentTime() {
  serial_debug.print("# Current time - ");
  if (hour(tt) < 10) {
    serial_debug.print("0");
  }
  serial_debug.print(hour(tt));
  serial_debug.print(":");
  if (minute(tt) < 10) {
    serial_debug.print("0");
  }
  serial_debug.print(minute(tt));
  serial_debug.print(":");
  if (second(tt) < 10) {
    serial_debug.print("0");
  }
  serial_debug.print(second(tt));
  serial_debug.print(" ");
  serial_debug.print(day(tt));
  serial_debug.print("/");
  serial_debug.print(month(tt));
  serial_debug.print("/");
  serial_debug.print(year(tt));
  serial_debug.println("("TZ")");

}

#if defined TOUCH_SCREEN_AVAILABLE
void touchCalibrate() {
  // showGraticule();
  for (uint8_t screenLayout = 0 ; screenLayout < 4 ; screenLayout += 1)
  {
    TFT.setRotation(screenLayout);
    TFT.setCursor(0, 10);
    TFT.print("  Press and hold centre circle ");
    TFT.setCursor(0, 20);
    TFT.print("   to calibrate touch panel.");
  }
  TFT.setRotation(PORTRAIT);
  TFT.drawCircle(myHeight / 2, myWidth / 2, 20, GRATICULE_COLOUR);
  TFT.fillCircle(myHeight / 2, myWidth / 2, 10, BEAM1_COLOUR);
  //delay(5000);
  readTouchCalibrationCoordinates();
  clearTFT();
}

void readTouchCalibrationCoordinates()
{
  int calibrationTries = 6000;
  int failCount = 0;
  int thisCount = 0;
  uint32_t tx = 0;
  uint32_t ty = 0;
  boolean OK = false;

  while (OK == false)
  {
    while ((myTouch.dataAvailable() == false) && thisCount < calibrationTries) {
      thisCount += 1;
      delay(1);
    }
    if ((myTouch.dataAvailable() == false)) {
      return;
    }
    // myGLCD.print("*  HOLD!  *", CENTER, text_y_center);
    thisCount = 0;
    while ((myTouch.dataAvailable() == true) && (thisCount < calibrationTries) && (failCount < 10000))
    {
      myTouch.calibrateRead();
      if (!((myTouch.TP_X == 65535) || (myTouch.TP_Y == 65535)))
      {
        tx += myTouch.TP_X;
        ty += myTouch.TP_Y;
        thisCount++;
      }
      else
        failCount++;
    }
    if (thisCount >= calibrationTries)
    {
      for (thisCount = 10 ; thisCount < 100 ; thisCount += 10)
      {
        TFT.drawCircle(myHeight / 2, myWidth / 2, thisCount, GRATICULE_COLOUR);
      }
      delay(500);
      OK = true;
    }
    else
    {
      tx = 0;
      ty = 0;
      thisCount = 0;
    }
    if (failCount >= 10000)
      // Didn't calibrate so just leave calibration as is.
      return;
  }
  serial_debug.print("# Calib x: ");
  serial_debug.println(tx/thisCount,HEX);
  serial_debug.print("# Calib y: ");
  serial_debug.println(ty/thisCount,HEX);
  // Change calibration data from here..
  // cx = tx / iter;
  // cy = ty / iter;
}

void readTouch() {

  if (myTouch.dataAvailable())
  {
    myTouch.read();
    // Note: This is corrected to account for different orientation of screen origin (x=0,y=0) in Adafruit lib from UTouch lib
    uint32_t touchY = myWidth - myTouch.getX();
    uint32_t touchX = myTouch.getY();
    // 
    serial_debug.print("# Touched ");
    serial_debug.print(touchX);
    serial_debug.print(",");
    serial_debug.println(touchY);
    TFT.drawPixel(touchX, touchY, BEAM1_COLOUR);
  }
}

#endif

void showCredits() {
  TFT.setTextSize(2);                           // Small 26 char / line
  //TFT.setTextColor(CURSOR_COLOUR, BEAM_OFF_COLOUR) ;
  TFT.setCursor(0, 50);
  TFT.print(" STM-O-Scope by Andy Hull") ;
  TFT.setCursor(0, 70);
  TFT.print("      Inspired by");
  TFT.setCursor(0, 90);
  TFT.print("      Ray Burnette.");
  TFT.setCursor(0, 130);
  TFT.print("      Victor PV");
  TFT.setCursor(0, 150);
  TFT.print("      Roger Clark");
  TFT.setCursor(0, 170);
  TFT.print(" and all at stm32duino.com");
  TFT.setCursor(0, 190);
  TFT.print(" CH1 Probe STM32F Pin [");
  TFT.print(analogInPin);
  TFT.print("]");
  TFT.setCursor(0, 220);
  TFT.setTextSize(1);
  TFT.print("     GNU GENERAL PUBLIC LICENSE Version 2 ");
  TFT.setTextSize(2);
  TFT.setRotation(PORTRAIT);
}

static inline int readBKP(int registerNumber)
{
  if (registerNumber > 9)
  {
    registerNumber += 5; // skip over BKP_RTCCR,BKP_CR,BKP_CSR and 2 x Reserved registers
  }
  return *(BKP_REG_BASE + registerNumber) & 0xffff;
}

static inline void writeBKP(int registerNumber, int value)
{
  if (registerNumber > 9)
  {
    registerNumber += 5; // skip over BKP_RTCCR,BKP_CR,BKP_CSR and 2 x Reserved registers
  }

  *(BKP_REG_BASE + registerNumber) = value & 0xffff;
}

