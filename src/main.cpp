#include <EEPROM.h>

#include <Arduino.h>

// neopixels_nodelay_button_eeprom
// by: Truglodite
// Non-blocking neopixel controller with a button to select patterns and wear leveling eeprom storage.
// (Requires the "Adafruit_NeoPixel" library. Code is inspired by this library's 'nodelay' example, modified for use on quadcopters)

// USEBETAFLIGHT works with betaflight pinio for control, using 1 to 2 channels on your transmitter.
// Pinio by default uses push/pull output; USEBETAFLIGHT disables pullups to work with both 3.3 and 5v 'arduino mcu's'.

// Pull the MODEPIN low to cycle through patterns, and push to high to "lock" the current pattern.
// The locked pattern is saved to eeprom and restored after reboots and toggling on/off.
// TOGGLEPIN turns all LEDs off when held low, and runs the previously locked pattern when pushed high.
// If the toggle pin is not used (left disconnected), the strip will always remain on (there is a blank pattern included that can still be used for off).

// This code stores the the locked pattern in eeprom only after a button release. EEPROM is read at boot, and the last locked pattern is restored.
// EEPROM writes use wear leveling methods to prolong the life of the chip.
// On the first boot, if EEPROM is all 0's or the first non-zero value is invalid, the default pattern is used and stored in address 0.

// Adding patterns: Keep all functions non-blocking, add a case to the switch code, and change TOTALPATTERNS to match the range of patterns available.
// Ex: If the last switch case is 17: TOTALPATTERNS 17

// ATINY85 notes:
// Add the url to the Arduino package manager in Preferences:
// http://drazzy.com/package_drazzy.com_index.json
// Tools/Boards/Boards Manager...
// Install the 'ATTinyCore by Spence Konde' board package.
// Tools/Board/ATTinyCore/ATtiny85 (Micronucleus/Digispark)
// Configure the code, then click Upload. The upload will pause and display a 60sec timer. 
// Plug the attiny85 into USB before the timer expires. It will proceed to flash the chip.

// Original example comments below....
// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)
//

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#include <EEPROM.h>

// USER CONFIGURATION ////////////////////////////////////////////////

// Pick only one board type uncomment it, and comment out the rest.
  //#define BEETLE
  #define DIGISPARK
  //#define NANO

#define LED_COUNT 74      // Total number of leds on the strip (pavopro = 77, pavo20 = 74)
#define BRIGHTNESS 255    // brightness level of the leds, from 0-255.
#define MODEDELAY 2000    // millis to wait between checking the toggle and mode pins (set to longer for modes that take longer to visualize/complete)
#define TOGGLEDELAY 200   // enough to debounce or otherwise delay toggling on/off of LED's
//#define USEBETAFLIGHT     // define when using FC pinio for control (push/pull signal), comment out if using physical buttons to ground.

// END USER CONFIGURATION ////////////////////////////////////////////////

// Pin definitions based on board selection
#ifdef BEETLE
  #define LED_PIN 9         // Pin used for the LED signal (9 for beetle, 2 for nano, 0 for DIGISPARK)
  #define MODEPIN A0        // Pin for the mode button/fc signal (A0 for beetle, 12 for nano, 1 for DIGISPARK)
  #define TOGGLEPIN 10      // Pin for the on/off button/fc signal (10 for beetle, 13 for nano, 2 for DIGISPARK)
#endif
#ifdef DIGISPARK
  #define LED_PIN 0         // Pin used for the LED signal (9 for beetle, 2 for nano, 0 for DIGISPARK)
  #define MODEPIN 1         // Pin for the mode button/fc signal (A0 for beetle, 12 for nano, 1 for DIGISPARK... note remove LED from middle of board)
  #define TOGGLEPIN 2       // Pin for the on/off button/fc signal (10 for beetle, 13 for nano, 2 for DIGISPARK)
#endif
#ifdef NANO
  #define LED_PIN 2         // Pin used for the LED signal (9 for beetle, 2 for nano, 0 for DIGISPARK)
  #define MODEPIN 12        // Pin for the mode button/fc signal (A0 for beetle, 12 for nano, 1 for DIGISPARK)
  #define TOGGLEPIN 13      // Pin for the on/off button/fc signal (10 for beetle, 13 for nano, 2 for DIGISPARK)
#endif

// Number of patterns available. Controls mode loop size (!!! increment when adding patterns to the code !!!)
#define TOTALPATTERNS 13

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // "NEO_GRB + NEO_KHZ800" works with Amazon 5V 160LED/m 5mm cobb led strips

unsigned long pixelPrevious = 0;        // Previous Pixel Millis
uint8_t       pattern = 0;              // Current Pattern Number
unsigned long pixelInterval = 50;       // Pixel Interval (ms)
int           pixelQueue = 0;           // Pattern Pixel Queue
int           pixelCycle = 0;           // Pattern Pixel Cycle
uint16_t      pixelNumber = LED_COUNT;  // Total Number of Pixels
unsigned long currentModeMillis = 8000; // Millis when the mode pin was last read (some FC's that hold pin low until blheli music finishes playing... init with 8000 to delay reading mode pin after boot.)
unsigned long currentToggleMillis = 0;  // Millis when the mode pin was last read
unsigned long currentMillis = 0;        // Storage of millis for each loop
bool          modeState = 0;            // High/Low state of the mode pin for current loop
bool          modeStatePrevious = 0;    // High/Low state of the mode pin from previous loop
bool          toggleState = 0;          // High/Low state of the toggle pin for current loop
bool          toggleStatePrevious = 0;  // High/Low state of the toggle pin from previous loop
bool          ledColorOff = 0;          // Switch to toggle between color on and color off (used for wipes, flashes, etc)
int           eepromAddress = 0;        // Storage for the currently used eepromAddress

void setup() {
  // No pullups needed on pins when using an FC for input. Betaflight pinio uses push/pull for output (actively drives both high and low).
  #ifdef USEBETAFLIGHT
    pinMode(MODEPIN,INPUT);          //  mode select push/pull signal.
    pinMode(TOGGLEPIN,INPUT);        //  toggle push/pull signal.
  #else
    pinMode(MODEPIN,INPUT_PULLUP);   //  mode select button to ground.
    pinMode(TOGGLEPIN,INPUT_PULLUP); //  toggle button to ground.
  #endif

  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS);

  while(!pattern) {        // Load a saved pattern from EEPROM
    pattern = EEPROM.read(eepromAddress);  // Read the address (starts at 0)
    // zero value at this address... increment address
    if(!pattern)  {                        
      eepromAddress++;
    }
    // eeprom is blank or contains an invalid first non-zero value, save default pattern to address 0  (usually just the first boot)
    if (eepromAddress >= EEPROM.length() || pattern > TOTALPATTERNS) { 
      eepromAddress = 0;
      pattern = 2;    // default pattern
      EEPROM.write(eepromAddress,pattern);
    }
  }
  // delay(1000); // a startup delay may be required by some FC's to give time for pin states to stabilize
}

// Some functions of our own for creating animated effects -----------------

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Solid Color for the whole strip
void solidColor(uint32_t color) {
  for(uint16_t i=0; i < pixelNumber; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
  return;
}
// Fill pixels one by one with a solid color, then color off, and repeating
void colorWipe(uint32_t color, int wait) {
  static uint16_t current_pixel = 0;
  pixelInterval = wait;                        //  Update delay time
  current_pixel++;
  if(current_pixel >= pixelNumber) {           //  Loop the pattern from the first LED
    current_pixel = 0;
    if(!ledColorOff) {
      ledColorOff = 1;
    }
    else  {
      ledColorOff = 0;
    }
  //  patternComplete = true;
  }
  if(!ledColorOff)  {
    strip.setPixelColor(current_pixel, color); //  Set pixel's color (in RAM)
  }
  else  {
    strip.setPixelColor(current_pixel, 0); //  Set pixel's color (in RAM)
  }
  strip.show();                                //  Update strip to match
}

// Theater-marquee-style chasing lights
void theaterChase(uint32_t color, int wait) {
  //static uint32_t loop_count = 0;
  static uint16_t current_pixel = 0;

  pixelInterval = wait;                   //  Update delay time

  strip.clear();

  for(uint16_t c=current_pixel; c < pixelNumber; c += 3) {
    strip.setPixelColor(c, color);
  }
  strip.show();

  current_pixel++;
  if (current_pixel >= 3) {
    current_pixel = 0;
  //  loop_count++;
  }

  //if (loop_count >= 10) {
  //  current_pixel = 0;
  //  loop_count = 0;
  //  patternComplete = true;
  //}
}

// 3 color theater chasing lights
void theaterChaseTricolor(uint32_t color1,uint32_t color2,uint32_t color3, int wait) {
  //static uint32_t loop_count = 0;
  static uint16_t current_pixel = 0;

  pixelInterval = wait;                   //  Update delay time

  strip.clear();

  for(uint16_t c=current_pixel; c < pixelNumber; c += 3) {
    strip.setPixelColor(c, color1);
  }
  for(uint16_t c=current_pixel + 1; c < pixelNumber; c += 3) {
    strip.setPixelColor(c, color2);
  }
  for(uint16_t c=current_pixel + 2; c < pixelNumber; c += 3) {
    strip.setPixelColor(c, color3);
  }
  strip.show();

  current_pixel++;
  if (current_pixel >= 3) {
    current_pixel = 0;
   // loop_count++;
  }

  //if (loop_count >= 10) {
  //  current_pixel = 0;
  //  loop_count = 0;
   // patternComplete = true;
  //}
}

// theater chasing with 3 color spaced clusters, adjustable width
void theaterChaseTricolorWidth(uint32_t color1,uint32_t color2,uint32_t color3,uint32_t width, int wait) {
  //static uint32_t loop_count = 0;
  static uint16_t current_pixel = 0;

  pixelInterval = wait;                   //  Update delay time

  strip.clear();
  for(int16_t c=current_pixel-6*width ; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++)  {
      strip.setPixelColor(c+i, color1);
    }
  }
  for(int16_t c=current_pixel-5*width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++) {
      strip.setPixelColor(c+i, 0);
    }
  }
  for(int16_t c=current_pixel-4*width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++)  {
      strip.setPixelColor(c+i, color2);
    }
  }
  for(int16_t c=current_pixel-3*width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++) {
      strip.setPixelColor(c+i, 0);
    }
  }
  for(int16_t c=current_pixel-2*width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++)  {
      strip.setPixelColor(c+i, color3);
    }
  }
  for(int16_t c=current_pixel-width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++) {
      strip.setPixelColor(c+i, 0);
    }
  }
  for(int16_t c=current_pixel ; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++)  {
      strip.setPixelColor(c+i, color1);
    }
  }
  for(int16_t c=current_pixel+width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++) {
      strip.setPixelColor(c+i, 0);
    }
  }
  for(int16_t c=current_pixel+2*width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++)  {
      strip.setPixelColor(c+i, color2);
    }
  }
  for(int16_t c=current_pixel+3*width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++) {
      strip.setPixelColor(c+i, 0);
    }
  }
  for(int16_t c=current_pixel+4*width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++)  {
      strip.setPixelColor(c+i, color3);
    }
  }
  for(int16_t c=current_pixel+5*width; c < pixelNumber; c += 6*width) {
    for(uint16_t i=0; i < width; i++) {
      strip.setPixelColor(c+i, 0);
    }
  }
  strip.show();

  current_pixel++;
  if (current_pixel >= 6*width) {
    current_pixel = 0;
  }
}

// theater chasing lights with 3 color clusters and 3 spaces between
void theaterChaseTricolorSpaces(uint32_t color1,uint32_t color2,uint32_t color3, int wait) {
  //static uint32_t loop_count = 0;
  static uint16_t current_pixel = 0;

  pixelInterval = wait;                   //  Update delay time

  strip.clear();

  for(uint16_t c=current_pixel; c < pixelNumber; c += 6) {
    strip.setPixelColor(c, color1);
  }
  for(uint16_t c=current_pixel + 1; c < pixelNumber; c += 6) {
    strip.setPixelColor(c, color2);
  }
  for(uint16_t c=current_pixel + 2; c < pixelNumber; c += 6) {
    strip.setPixelColor(c, color3);
  }
  for(uint16_t c=current_pixel + 3; c < pixelNumber; c += 6) {
    strip.setPixelColor(c, 0);
    strip.setPixelColor(c+1, 0);
    strip.setPixelColor(c+2, 0);
  }
  strip.show();

  current_pixel++;
  if (current_pixel >= 6) {
    current_pixel = 0;
  }
}

// Rainbow cycle, 1 color step between each LED
void rainbow(uint8_t wait) {
  if(pixelInterval != wait)
    pixelInterval = wait;                   
  for(uint16_t i=0; i < pixelNumber; i++) {
    strip.setPixelColor(i, Wheel((i + pixelCycle) & 255)); //  Update delay time  
  }
  strip.show();                             //  Update strip to match
  pixelCycle++;                             //  Advance current cycle
  if(pixelCycle >= 256)
    pixelCycle = 0;                         //  Loop the cycle back to the begining
}

// Rainbow cycle with the complete rainbow distributed on the strip
void rainbowFull(uint8_t wait) {
  if(pixelInterval != wait)
    pixelInterval = wait;                   
  uint16_t i;
  for(i=0; i< strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + pixelCycle) & 255));
  }
  strip.show();                             //  Update strip to match
  pixelCycle++;                             //  Advance current cycle
  if(pixelCycle >= 256)
    pixelCycle = 0;                         //  Loop the cycle back to the begining
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  if(pixelInterval != wait)
    pixelInterval = wait;                   //  Update delay time  
  for(uint16_t i=0; i < pixelNumber; i+=3) {
    strip.setPixelColor(i + pixelQueue, Wheel((i + pixelCycle) % 255)); 
  }
  strip.show();
  for(uint16_t i=0; i < pixelNumber; i+=3) {
    strip.setPixelColor(i + pixelQueue, strip.Color(0, 0, 0));
  }      
  pixelQueue++;                           //  Advance current queue  
  pixelCycle++;                           //  Advance current cycle
  if(pixelQueue >= 3)
    pixelQueue = 0;                       //  Loop
  if(pixelCycle >= 256)
    pixelCycle = 0;                       //  Loop
}

// Flashing Solid Color for the whole strip
void flashingColor(uint32_t color, uint8_t wait) {
  pixelInterval = wait;                   //  Update delay time
  if(pixelCycle > 1) pixelCycle = 0;
  if(pixelCycle)  {
    pixelCycle = 0;
    for(uint16_t i=0; i < pixelNumber; i++) {
      strip.setPixelColor(i, 0);
    }
  }
  else  {
    pixelCycle = 1;
    for(uint16_t i=0; i < pixelNumber; i++) {
      strip.setPixelColor(i, color);
    }
  }
  strip.show();
  return;
}

// Alternating Solid Color Strip Halves (half of strip color1, other half color2)
void emergency(uint32_t color1,uint32_t color2, uint8_t wait) {
  pixelInterval = wait;                   //  Update delay time
  if(pixelCycle > 1) pixelCycle = 0;
  if(pixelCycle)  {
    pixelCycle = 0;
    for(uint16_t i=0; i < pixelNumber/2; i++) {
      strip.setPixelColor(i, color1);
    }
    for(uint16_t i=pixelNumber/2; i < pixelNumber; i++) {
      strip.setPixelColor(i, color2);
    }
  }
  else  {
    pixelCycle = 1;
    for(uint16_t i=0; i < pixelNumber/2; i++) {
      strip.setPixelColor(i, color2);
    }
    for(uint16_t i=pixelNumber/2; i < pixelNumber; i++) {
      strip.setPixelColor(i, color1);
    }
  }
  strip.show();
  return;
}

// Alternating Solid Color Bands... choose width of the bands
void alternatingBands(uint32_t color1,uint32_t color2, uint8_t width, uint8_t wait) {
  pixelInterval = wait;                   //  Update delay time
  if(pixelCycle > 1) pixelCycle = 0;
  if(pixelCycle)  {
    pixelCycle = 0;
    for(uint16_t i=0; i < pixelNumber; i=i) {
      for(uint16_t j=0; j < width ; j++) {
        if(i < pixelNumber) {
          strip.setPixelColor(i, color1);
          i++;
        }
      }
      for(uint16_t j=0; j < width ; j++) {
        if(i < pixelNumber) {
          strip.setPixelColor(i, color2);
          i++;
        }
      }
    }
  }
  else  {
    pixelCycle = 1;
    for(uint16_t i=0; i < pixelNumber; i=i) {
      for(uint16_t j=0; j < width ; j++) {
        if(i < pixelNumber) {
          strip.setPixelColor(i, color2);
          i++;
        }
      }
      for(uint16_t j=0; j < width ; j++) {
        if(i < pixelNumber) {
          strip.setPixelColor(i, color1);
          i++;
        }
      }
    }
  }
  strip.show();
  return;
}

// DON'T FORGET to add a case when adding a new pattern function!!!

void loop() {
  currentMillis = millis();                     //  Update current time
  
  // Read toggle pin and update accordingly, every TOGGLETIME milliseconds
  if(currentMillis - currentToggleMillis > TOGGLEDELAY ) {
    currentToggleMillis = currentMillis;
    toggleStatePrevious = toggleState;  // Save toggle pin then read new value
    toggleState = digitalRead(TOGGLEPIN);
    if(!toggleState) {    // toggle pin is low, turn off the strip
      pattern = 1;
    }
    else if(!toggleStatePrevious) {  // toggle was pin released, reload the pattern saved in EEPROM
      pattern = EEPROM.read(eepromAddress);
    }
    else  return;
  }

  // Read inputs and update pattern  every MODEDELAY milliseconds
  if(currentMillis - currentModeMillis > MODEDELAY ) {
    currentModeMillis = currentMillis;    // reset timer
    modeStatePrevious = modeState;    // save previous mode pin state
    modeState = digitalRead(MODEPIN);
    if(!modeState) {    // mode pin is low
      pattern++;
      if(pattern > TOTALPATTERNS) {   // repeat cycling through patterns, including off (in case toggle is not used)
        pattern = 1;
      }
    }
    if(modeState && !modeStatePrevious && pattern != EEPROM.read(eepromAddress)) { // mode pin is released, and the selected pattern is different than the one stored in EEPROM... save the new pattern
      eepromAddress++;  // increment eeprom for wear leveling
      if (eepromAddress == EEPROM.length()) { // check for overflow address, reset to address 0 if true
        eepromAddress = 0;
      }
      EEPROM.write(eepromAddress-1, 0); // write 0 to old eeprom address, then write to the new address
      EEPROM.write(eepromAddress,pattern);  // no need for a delay here to prevent eeprom wear, since MODEDELAY repeats slow enough to debounce a button
    }
  }


  // Update pixels when ready
  if(currentMillis - pixelPrevious >= pixelInterval) {        //  Check for expired time
    pixelPrevious = currentMillis;                            //  Run current frame
    switch(pattern) {                                   
      // Some patterns take the format:
      // [strip.Color(red, green, blue), ..., millis between frames].
      // Change colors of these patterns by editing the color numbers to any value between 0-255.
      // See this page to get color codes: https://www.google.com/search?q=rgb+color+picker
      // The last number is milliseconds between frames. Edit this number to change a pattern's speed (lower is faster).
      // Some patterns have additional inputs (info on these inputs is found in the comments)
      // You can copy/paste/modify patterns to make variations, or create custom pattern functions as you wish.
      // Just remember to increment the TOTALPATTERNS definition at the top, and add a switch case below so you can access them.
      
      default: { // this case should never happen...
        flashingColor(strip.Color(255, 0, 0), 10);  // fast flashing solid red ERROR!!!
        break;
      }
      case 1: { // all off
        strip.clear();
        strip.show();
        break;
      }
      case 2: { // default pattern (on first boot usually... also later boots if no mode changes are saved)
        rainbowFull(1);  // same as rainbow, but all colors shown at once
        break;
      }
      case 3: { // Red white and blue
        theaterChaseTricolor(strip.Color(255, 0, 0),strip.Color(255, 255, 255),strip.Color(0, 0, 255), 50);
        break;
      }
      case 4: { // Rainbow incrementing one color per led (256 colors)
        rainbow(5);
        break;     
      }
      case 5: { // Rainbow-enhanced theaterChase variant
        theaterChaseRainbow(50);
        break;
      }
      case 6: { // Green wipe
        colorWipe(strip.Color(0, 255, 0), 10);
        break;        
      }
      case 7: { // Red white and blue clusters
        theaterChaseTricolorSpaces(strip.Color(255, 0, 0),strip.Color(255, 255, 255),strip.Color(0, 0, 255), 25);
        break;
      }
      case 8: { // solid green
        solidColor(strip.Color(255, 255, 255)); 
        break;
      }
      case 9: { // flashing solid green
        flashingColor(strip.Color(255, 255, 255), 200); 
        break;
      }
      case 10: { // Red theater chase
        theaterChase(strip.Color(255, 0, 0), 50);
        break;
      }
      case 11: { // alternating halves, red and blue
        emergency(strip.Color(255, 0, 0),strip.Color(0, 0, 255), 100); 
        break;
      }
      case 12: { // alternating yellow & blue 10 pixel wide bands, 100msec frame period
        alternatingBands(strip.Color(245, 200, 66),strip.Color(0, 0, 255), 10, 100); 
        break;
      }
      case 13: { // theater chase, 3 color bands, adjustable width (sacmob Y P G)
        theaterChaseTricolorWidth(strip.Color(168, 117, 0),strip.Color(255, 14, 89),strip.Color(43, 198, 57),3,20);
        break;
      }
    } // DON'T FORGET to increment TOTALPATTERNS definition when adding a new pattern case!!!
  }
}

