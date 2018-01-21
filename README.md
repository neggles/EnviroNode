# EnviroNode
ESP8266-based environmental monitoring node software, intended for a WeMos D1 + SHT30 shield + 64x48 OLED shield.
Created by [Andrew Holmes](neg2led@gmail.com). Licensed under [Creative Commons Attribution-ShareAlike 4.0](https://creativecommons.org/licenses/by-sa/4.0/)
Contents heavily stolen from Steve Chamberlin at Big Mess O' Wires; his post [here](http://bit.ly/2D7xqt7) contains the code I began from.

### Code features;
* NTP client and WiFi connectivity
* Displays current time and day of week
* Supports SHT30 and BMP085 sensors, displays temperature for both and;
* SHT30: Dew point or relative humidity (currently hardcoded)
   + BMP085: Atmospheric pressure
   + Connects to Losant dashboard to log values
   + Timezone support with automatic DST settings
   + Display both dewpoint and RH for SHT30 alternating every 5 seconds
### New features I'm planning
- Maybe geoIP for auto-timezone? Is there a library for this?
- use SPIFFS to store JSON config
- WiFi signal strength gauge (3-step)
- Sensor type auto-detect?

Most libraries are built-in or available from the library manager as of Arduino 1.8.5 - details are in individual library   comments
The SSD1306 library is semi-custom & courtesy of Mark Causer & is available [on github here](https://github.com/mcauser/Adafruit_SSD1306/tree/esp8266-64x48)
It should be included with this sketch, as it was somewhat hard to find - f you remix this and release it elsewhere, please include it also.

**_Please note that the Losant library requires ArduinoJson from the library manager to function as it uses this internally!_**
**_See Losant Arduino docs at https://docs.losant.com/getting-started/boards/getting-started-with-arduino-wifi-101/_**
