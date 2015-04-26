# STM32-O-Scope
STM32F103 based minimalist oscilloscope. 

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

The above will allow you to build a very modest Oscilloscope with the following rough spec. 

Input Voltage: Full scale input voltage 0 to 3.3V (or whatever voltage you set for the ADC on your STM32F103XXXX board). 

Sampling: variable from about 6uS between samples to near infinite. 

Resolution: Limited by the resolution of the ADC, quoted at 4096 samples over the full scale range, or approximately 0.000805664V per pixel. i.e. 8uV per pixel. 

Samples: Limited by the RAM on your STM32F103 for the STM32F103C8T6 you should be able to store >6 thousand samples, which surprisingly makes this better than some low end commercial offerings. 

In summary you can graph signals up to a theoretical maximum of around 83Khz although better resolution (up to 2 Megasamples per second is possible, but would require some changes to the code). 

WARNING: This scope is not protected against excessive voltage in any way, add a high impedance attenuator and front end if you want to do anything outside of the limits of the STM32F103 otherwise you will release the magic smoke. 

