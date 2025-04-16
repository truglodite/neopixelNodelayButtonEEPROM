# neopixelNodelayButtonEEPROM: Yet another neopixel control
This code was written for use with neopixel LED strips mounted on an FPV quadcopter. It has 2 inputs for use with betaflight PINIO to enable on/off as well as pattern control from the transmitter. Some patterns are already included, but the idea is to customize colors and patterns to suit your unique needs.

## Features
* Controls far more LED's than betaflight alone (which is limited to 64 leds)
* Customizeable colors and patterns using open source code
* Compatible with common WS28XX leds and led strips
* Transmitter controlled LEDs on/off
* Transmitter controlled LED patterns/colors
* EEPROM storage of last selected pattern, with wear leveling
* Runs on a variety of Arduino hardware (Beetle, Attiny85, etc...)

## Currently Compatible Hardware
* Arduino Beetle
* Digispark Attiny85
* ...more to come as needed.

## Instructions
### Betaflight CLI

Control of the LED’s is done through betaflight pinio. The Pavo Pro fc should come already configured with a USER1 mode, which toggles the TX6 pin on/off. This pin is already prepared for toggling the pin on the beetle board which will turn the LEDs on and off. You need to add a USER2 for mode control via transmitter. In betaflight CLI enter the lines (for the betafpv 2-3s 20A aio only!):

'''
resource SERIAL_RX 6 none
resource PINIO 3 c07
set pinio_box = 40,41,255,255
save
'''

Next in the betaflight modes tab, select ranges for both USER1 and USER2, using different channels on your Tx. Verify that you can individually enable/disable USER1 and USER2 from your tx switches. Note in the above image the User2 logic is shown backwards; it’s actually high = lock pattern, and low = change pattern.

Once it’s all wired up, and both betaflight pinio’s are working properly, the LED strip should now be fully functional. If you would like to modify colors, change speed of patterns, use LED strips with more or less LEDs, read the sections below.

## Contribute
The main repository lives on [Github](https://github.com/truglodite/Wire-Wiggler-Firmware).

Anyone is welcome to use/edit/share this code as they please as long as it is within the terms of the GPLv3 license. We welcome everyone to join in on the fun with pull requests, bug reports, suggestions, etc. Stay on the air and have fun!

## License
Licensed under the [GPLv3](LICENSE)
