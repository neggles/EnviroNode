/*
   ESP8266 / WeMos D1 Mini-based IOT environmental logging node software, Created by Andrew Holmes <neg2led@gmail.com>; last updated 13/01/2018.
   Licensed under Creative Commons Attribution-ShareAlike 4.0 https://creativecommons.org/licenses/by-sa/4.0/
   Contents heavily stolen from Steve Chamberlin at Big Mess O' Wires; his post here http://bit.ly/2D7xqt7 contains the code I began from.

   Code features;
    - NTP client and WiFi connectivity
    - Displays current time and day of week
    - Supports SHT30 and BMP085 sensors, displays temperature for both and;
       + SHT30: Dew point or relative humidity (currently hardcoded)
       + BMP085: Atmospheric pressure
    - Connects to Losant dashboard to log values
    - Timezone support with automatic DST settings
    - Display both dewpoint and RH for SHT30 alternating every 5 seconds

   New features I'm planning ;
    - Maybe geoIP for auto-timezone? Is there a library for this?
    - use SPIFFS to store JSON config
    - WiFi signal strength gauge (3-step)
    - Sensor type auto-detect?

    Most libraries are built-in or available from the library manager as of Arduino 1.8.5 - details are in individual library comments
    The SSD1306 library is semi-custom & courtesy of Mark Causer & iss available here https://github.com/mcauser/Adafruit_SSD1306/tree/esp8266-64x48
    It should be included with this sketch, as it was somewhat hard to find - f you remix this and release it elsewhere, please include it also.

    !!! Please note that the Losant library requires ArduinoJson from the library manager to function as it uses this internally!   !!!
    !!! See Losant Arduino docs at https://docs.losant.com/getting-started/boards/getting-started-with-arduino-wifi-101/            !!!
*/
#define SKETCH_VER 2.2.3 // seems I've rewritten this a few times now

// Networking & time libraries
#include <ESP8266WiFi.h>              // built-in "ESP8266WiFi"
#include <ESP8266HTTPClient.h>        // built-in "ESP8266HTTPClient"
#include <WiFiUDP.h>                  // built-in "WiFi" (for 'duino official wifi shield) - used for NTP funcs
#include <ArduinoJson.h>              // Library manager - ArduinoJson
#include <Adafruit_SHT31.h>           // Library manager - Adafruit SHT31 Library
#include <Adafruit_BMP085.h>          // Library manager - Adafruit BMP085 Library (not "unified")
#include <Adafruit_GFX.h>             // Library manager - Adafruit GFX Library
#include <gfxfont.h>                  // ^^
#include <Fonts/FreeSans9pt7b.h>      // ^^
#include <Fonts/FreeSansBold9pt7b.h>  // ^^
#include <Losant.h>                   // External custom library for Losant - see main desc. above for details
#include <Time.h>                     // Paul Stoffregen's Time library - https://github.com/PaulStoffregen/Time
#include <Timezone.h>                 // Jack Christensen's Timezone library - https://github.com/JChristensen/Timezone
#include "Adafruit_SSD1306.h"         // This is the custom SSD1306 module for AdafruitGFX mentioned above, courtesy of Mark Causer.
//#include "FS.h"                     // ESP8266 SPIFFS module - not using this yet

// WiFi credentials.
const char* wifi_ssid = "your_wifi_ssid";
const char* wifi_psk = "your_wifi_psk";

// Losant device ID & API credentials
const char* LOSANT_DEVICE_ID = "your_losant_devid_1"; // node 1
//const char* LOSANT_DEVICE_ID = "your_losant_devid_2"; // node 2
const char* LOSANT_ACCESS_KEY = "your_losant_access_key";
const char* LOSANT_ACCESS_SECRET = "your_losant_access_secret";

// Select your sensor
#define SHT30
//#define BMP085

// Temperature scale
const char tempScale = 'C';
//char tempScale = 'F';

// Time settings; NTP server and local port
const char ntpServerName[] = "au.pool.ntp.org";
unsigned int localPort = 6144;

// Timezone. {abbrev, week, dow, month, hour, offset}
// Australian Eastern Time (Canberra, Melbourne, Sydney) - change at your leisure
TimeChangeRule aEDT = {"AEDT", First, Sun, Oct, 2, 660};    // UTC + 11 hours
TimeChangeRule aEST = {"AEST", First, Sun, Apr, 3, 600};    // UTC + 10 hours
Timezone myTZ(aEDT, aEST);
TimeChangeRule *tcr; // pointer for timechangerule

/* configuration section done */

// Set up sensors
#ifdef SHT30
Adafruit_SHT31 sht30;
//Adafruit_SHT31 sht30 = Adafruit_SHT31();
float temperatureV;
float humidityV;
float dewpointV;
#endif
#ifdef BMP085
Adafruit_BMP085 bmp085;
float temperatureV;
float pressureV;
float altitudeM = 100.0f;
#endif

// OLED setup, reset on GPIO0
#define OLED_RESET 0
Adafruit_SSD1306 display(OLED_RESET);
// This doesn't have to be WiFiClientSecure, but why send data unencrypted these days?
WiFiClientSecure wifiClient;
// time vals and WiFiUDP instance
time_t utc,local;
WiFiUDP ntpUDP;
// Losant device
LosantDevice device(LOSANT_DEVICE_ID);

int lastPollTime = 0;
int lastUpdateTime = 0;
const char degSym = char(0xF7); // degrees symbol from adafruit font

// Main initialization sequence
void setup() {
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(14, 16);
  display.print("hello!");
  display.display();
  delay(125);
  
#ifdef SHT30
  sht30.begin(0x45);
#endif
#ifdef BMP085
  bmp085.begin();
#endif

  display.setCursor(8, 24);
  display.print("init OK!");
  display.display();
  Serial.println();
  Serial.println("Device init OK!");
  delay(250);

  initWifi();
  initLosant();

  // Pass the command handler function to the Losant device.
  device.onCommand(&handleCommand);

  // Get our time on
  ntpUDP.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
//  setTime(int(getNtpTime)); // not sure if need
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (device.connected()) {
      // wifi up & losant connected, do normal loop stuff
      device.loop();
      if (millis() - lastPollTime >= 1000) {
        lastPollTime = millis();
        utc = now();
        local = myTZ.toLocal(utc);
        pollSensors();
        updateDisplay();
      }
      if (utc - lastUpdateTime >= 15) {
        lastUpdateTime = utc;
        updateLosant();
      }
    } else {
      // wifi up but losant down, fix losant
      Serial.println("Losant not up, reconnecting");
      display.clearDisplay();
      if (! initLosant()) {
        Serial.println("Losant failed to reconnect, waiting 3s");
        delay(3000); 
      }
    }
  } else {
    // wifi down, reconnect - next loop will fix losant
    Serial.println("Wifi not up, connecting");
    display.clearDisplay();
    if (! initWifi()) {
      Serial.println("Wifi failed to reconnect, waiting 3s");
      delay(3000); 
    }
  }
}


// connects to WiFi, returns true on success and false on fail)
bool initWifi() {
  int timeout = 10000;
  int startTime = millis(); // for checking timeout state
  display.clearDisplay(); // reinit display
  display.setFont();
  display.setCursor(0,0);

  Serial.println("WiFi init; connecting to " + String(wifi_ssid));
  display.println("WiFi SSID:");
  display.println(String(wifi_ssid));
  display.display();

  if (WiFi.status() != WL_CONNECTED) {
    // Reinitialize wifi driver to fix https://github.com/esp8266/Arduino/issues/2186
    WiFi.persistent(false);
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    // Attempt to connect
    WiFi.begin(wifi_ssid, wifi_psk);

    while (millis() - startTime < timeout) {
      switch (WiFi.status()) {
        case WL_CONNECTED:
          Serial.println();
          Serial.println("Success! Connected to " + String(wifi_ssid));
          display.println("OK!");
          display.display();
          delay(100);
          return true;
        case WL_CONNECT_FAILED:
          Serial.println();
          Serial.println("ERR! WiFi reports bad PSK. Check config.");
          display.println("bad pwd :(");
          display.display();
          WiFi.mode(WIFI_OFF);
          delay(3000);
          return false;
        default:
          display.print(".");
          display.display();
          Serial.print(".");
          delay(500);
      }
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("ERR! WiFi timed out. Check config.");
      WiFi.mode(WIFI_OFF);
      delay(3000);
      return false;
    }
  }
  return false;
}

// connects to Losant, returns true on success and false on fail
bool initLosant() {
  int timeout = 10000;
  int startTime = millis(); // for checking timeout state
  display.setCursor(0,30);
  display.print("Cloud:");
  display.display();
  Serial.print("Losant init, connecting...");

  // Connect the device instance to Losant using TLS encryption.
  device.connectSecure(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);
  if(device.connected()) {
    display.println("OK!");
    display.display();
    Serial.println("OK!");
    delay(500);
    return true;
  }
  display.println("fail!");
  display.display();
  Serial.println("fail!");
  WiFi.mode(WIFI_OFF);
  delay(3000);
  return false;
}

// Command callback function - invoked on receiving a command from Losant
void handleCommand(LosantCommand *command) {
  Serial.print("Command received: ");
  Serial.println(command->name);

  // Optional command payload. May not be present on all commands.
  JsonObject& payload = *command->payload;

  // Perform action specific to the command received.
  if (strcmp(command->name, "start recording") == 0) {
    int resolution = payload["resolution"];
  }
}

bool pollSensors() {
#ifdef SHT30
  temperatureV = sht30.readTemperature();
  humidityV = sht30.readHumidity();
  dewpointV = 243.04 * (log(humidityV / 100) + ((17.625 * temperatureV) / (243.04 + temperatureV))) /
              (17.625 - log(humidityV / 100) - ((17.625 * temperatureV) / (243.04 + temperatureV)));
  if (tempScale == 'F') {
    temperatureV = (temperatureV * 9 / 5) + 32;
    dewpointV = (dewpointV * 9 / 5) + 32;
  }
#endif
#ifdef BMP085
  temperatureV = bmp085.readTemperature();
  pressureV = (float)bmp085.readSealevelPressure(altitudeM) / 3386.389f; // convert Pascals to in-Hg
  if (tempScale == 'F') {
    temperatureV = (temperatureV * 9 / 5) + 32;
  }
#endif
}

void updateDisplay() {
  const char degSym = char(0xF7); // degree symbol from font
  const int line1y = 15; // Y-heights for txt lines
  const int line2y = 28;
  
  display.clearDisplay(); // clear buffer
  display.setFont(&FreeSansBold9pt7b); // temperature font
  String temperatureString = String(temperatureV);
  if(temperatureString.length() <= 4) {
    display.setCursor(18, line1y);
  } else {
    display.setCursor(8, line1y);
  }
  display.print(temperatureV, 1);
  display.setFont();
  display.setCursor(44, 2);
  display.print(char(0xF7));
  display.print(tempScale);

#ifdef SHT30
  if (second() % 10 >= 5 ) {
    //display.setCursor(0, line2y - 6);
    display.setCursor(0, line2y);
    display.println("DP");
    //display.println("P");
    display.setFont(&FreeSans9pt7b);
    String dewpointString = String(dewpointV);
    if(dewpointString.length() <= 4) {
      display.setCursor(26, line2y + 6);
    } else {
      display.setCursor(16, line2y + 6);
    }
    display.print(dewpointV, 1);
    display.setFont();
    display.setCursor(54, line2y - 6);
    display.print(char(0xF7));
  } else {
    display.setCursor(0, line2y - 6);
    display.println("RH");
    //display.println("H");
    display.setFont(&FreeSans9pt7b);
    display.setCursor(16, line2y + 6);
    display.print(humidityV, 1);
    display.setFont();
    display.setCursor(54, line2y);
    display.print("%");
  }
#endif
#ifdef BMP085
  display.setFont(&FreeSans9pt7b);
  display.setCursor(0,line2y+6);
  display.print(pressureV, 2);
  display.setFont();
  display.setCursor(50,line2y);
  display.print("in");
#endif

  display.setFont();
  display.setCursor(0, 41);
  //display.print("Fri 10:33A");
  char dateTimeBuf[16];
  char* dayName = dayShortStr(weekday(local));
  int hour24 = hour(local);
  int hour12 = hour24 == 0 ? 12 : hour24 > 12 ? hour24 - 12 : hour24;
  if (utc % 2) {
    sprintf(dateTimeBuf, "%s %s%d:%02d%c", dayName, hour12 < 10 ? " " : "", hour12, minute(), hour24 < 12 ? 'A' : 'P');
  } else {
    sprintf(dateTimeBuf, "%s %s%d %02d%c", dayName, hour12 < 10 ? " " : "", hour12, minute(), hour24 < 12 ? 'A' : 'P');
  }
  display.print(dateTimeBuf);
  display.display();
}

void updateLosant() {
  // create a buffer to hold the state report data
  StaticJsonBuffer<250> jsonBuffer;
  JsonObject& state = jsonBuffer.createObject();
  state["temperature"] = temperatureV;
#ifdef SHT30
  state["humidity"] = humidityV;
  state["dew-point"] = dewpointV;
#endif
#ifdef BMP085
  state["pressure"] = pressureV;
#endif
  // Report the state to Losant.
  device.sendState(state);
}

/*-------- NTP code ----------*/

#define NTP_PACKET_SIZE 48 // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (ntpUDP.parsePacket() > 0) ; // discard any previously received packets
  //Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  //Serial.print(ntpServerName);
  //Serial.print(": ");
  //Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = ntpUDP.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      //Serial.println("Receive NTP Response");
      ntpUDP.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL;
    }
  }
  //Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  ntpUDP.beginPacket(address, 123); //NTP requests are to port 123
  ntpUDP.write(packetBuffer, NTP_PACKET_SIZE);
  ntpUDP.endPacket();
}
