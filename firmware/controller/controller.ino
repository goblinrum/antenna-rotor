#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Sgp4.h>
#include <SPI.h>
#include <Ticker.h>
#include <Wire.h>
#include <WiFi.h>
#include <string.h>
#include "time.h"
#include "esp_sntp.h"

// user Serial 2 for GPS
#define RXD2 16
#define TXD2 17

// Wifi definitions for time sync
const char* ssid = "ESP32-Access-Point";
const char* password = "silentbolt977";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -8 * 3600;
const int   daylightOffset_sec = 0;

// screen definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// gps definitons
char gps[64];
int count = 0; 

// processing gps data
char *token;
const char delimiter[2] = ","; 

// positional arguments
float lat = 0; // latitude
float lon = 0; // longitude
float alt = 0; // altutide
float azi = 0; // azimuth
float ele = 0; // elevation
float satLat = 0; // satellite lat
float satLon = 0; // satellite long
float satAlt = 0; // satellite altitude

// TLE data
char satname[] = "ISS (ZARYA)";
char tle_1[70];
char tle_2[70];

// SGP4 information
Sgp4 sat;
Ticker tkSecond;
unsigned long unixtime = 0;
int timezone = -8 ;  //utc - 8
int framerate;
int year; int mon; int day; int hr; int minute; double sec;
bool done_setup = false;

// current state machine state
int state = 1; 

void setup() {
    // set up serial connection to computer here
    Serial.begin(9600); // Starts the serial communication to the computer
    // setup gps serial
    Serial2.begin(9600); // Serial for the GPS module, will need to define TX and RX for ESP32
    // setup motors here

    // setup wifi once
    // setup_wifi();

    // setup screen
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    display.drawPixel(10, 10, SSD1306_WHITE);
    display.display();
    display.clearDisplay();
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner
    display.println("Welcome");
    display.display();
}

/*
State machine management on loop 
*/
void loop() {
    switch (state) {
      // Step 1: read the gps data (leave in acquring data mode until fix is found)
      case 1: setup_gps(); break;
      // Step 2: 
      //  - Option 1: use the gps data to directly request API
      //  - Option 2: send GPS data to computer, and have computer request API
      case 2: process_tle(); break;
      // Step 3: use the API data to calculate the position needed
      // uses library like this: https://github.com/Hopperpop/Sgp4-Library
      // this enters tracking mode
      case 3: predict(); break;
      // Step 4: Actually move the motors to the desired azimuth/elevation
      case 4: track_motor(); break;
      // Step 5: send the motor data to the computer
      case 5: update_server(); break;
      default: Serial.println("Default case");
    }
}

/* 
Read from GPS until a fix is acquired. 
Transitions into read_gps()
*/
void setup_gps() {
  // read from GPS
  bool fix = false;
  if (Serial2.available()) {
    while (Serial2.available()) {
      // read one line at a time until a valid GPRMC line is found and "V" is not in the line
      String line = Serial2.readStringUntil('\n');
      // Serial.println(line);
      delay(1000);
      if (!fix && line.indexOf("GPRMC") != -1 && line.indexOf("V") == -1) {
        fix = true;
        // Serial.println("fix acquired");
      } else if ((fix && line.indexOf("GPGGA") != -1) || 
                  (line.indexOf("GPRMC") != -1 && line.indexOf(",,,,") == -1) || 
                  (line.indexOf("GPGGA") != -1 && line.indexOf(",,,") == -1)) {
        clearBuffer();
        line.toCharArray(gps, 64);
        display.clearDisplay();
        display.println(gps);
        display.display();
        read_gps();
        state = 2;
        break;
      }
    }
  }
}

/* 
Process the TLE from the computer and saves it
Transitons to predict()
*/
void process_tle() {
  if (Serial.available()) {
    String input = Serial.readString();
    // then split input by \n and copy results to tle_1 and tle_2
    int ind = input.indexOf('\n');
    String line1 = input.substring(0, ind);
    String line2 = input.substring(ind + 1);
    line1.toCharArray(tle_1, 70);
    line2.toCharArray(tle_2, 70);
    display.clearDisplay();
    display.println("TLE received");
    display.display();
    state = 3;
  }
}

/* 
Performs first time setup if not done yet, then loops to do the actual prediction
*/
void predict() {
  if (done_setup) { // predict based on new unixtime
    unixtime = getTime();
    sat.findsat(unixtime);
    framerate += 1;
    state = 4;
  } else { // first time setup
    sat.site(lat, lon, alt);
    sat.init(satname, tle_1, tle_2);
    double jdC = sat.satrec.jdsatepoch;
    invjday(jdC , timezone, true, year, mon, day, hr, minute, sec);
    // Serial.println("Epoch: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
    // Serial.println();
    tkSecond.attach(1,Second_Tick);
    done_setup = true;
  }
}

/* 
Moves the motor to input azimuth and elevation
*/
void track_motor() {
  // TODO
  state = 5;
}

/*
Updates the server on the new position and coordinates 
*/
void update_server() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input == "get_coords") {
      // send satellite coordinates
      Serial.println(String(satLat) + "," + String(satLon) + "," + String(satAlt) + "," + String(azi) + "," + String(ele));
    }
  }
}

////////////////////// Utility Functions //////////////////////

/* 
Example prediction function taken from: 
https://github.com/Hopperpop/Sgp4-Library/blob/master/examples/Sgp4Tracker/Sgp4Tracker.ino
*/
void Second_Tick() {
  unixtime += 1;
       
  invjday(sat.satJd , timezone,true, year, mon, day, hr, minute, sec);
  // Serial.println(String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
  // Serial.println("azimuth = " + String( sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
  // Serial.println("latitude = " + String( sat.satLat) + " longitude = " + String( sat.satLon) + " altitude = " + String( sat.satAlt));

  azi = sat.satAz;
  ele = sat.satEl;
  satLat = sat.satLat;
  satLon = sat.satLon;
  satAlt = sat.satAlt;

  // switch(sat.satVis){
  //   case -2:
  //     Serial.println("Visible : Under horizon");
  //     break;
  //   case -1:
  //     Serial.println("Visible : Daylight");
  //     break;
  //   default:
  //     Serial.println("Visible : " + String(sat.satVis));   //0:eclipsed - 1000:visible
  //     break;
  // }

  // Serial.println("Framerate: " + String(framerate) + " calc/sec");
  // Serial.println();
     
  framerate=0;
}

/* 
Uses the GPS data and parses the $GPGGA field into
latitude, longitude, and altidude

Transitions into process_tle()
*/
void read_gps() {
  token = strtok(gps, delimiter); // Initial tokenization
  char *end;
  for (int i = 0; i < 12; i++) { // Loop to reach the required tokens
    token = strtok(NULL, delimiter);
    if (i == 1 || i == 2) { // Latitude
      if (strstr(token, "S") == NULL && strstr(token, "N") == NULL) { // if it's a number
        lat = strtof(token, &end);
        lat /= 100; // convert to degrees
      } else if (strstr(token, "S") != NULL) { // if it's in the Southern Hemisphere
        lat *= -1;
      }
    } else if (i == 3 || i == 4) { // Longitude
      if (strstr(token, "W") == NULL && strstr(token, "E") == NULL) { // if it's a number
        lon = strtof(token, &end);
        lon /= 100; // convert to degrees
      } else if (strstr(token, "W") != NULL) { // if it's in the West
        lon *= -1;
      }
    } else if (i == 8 || i == 9) {
      if (strstr(token, "M") == NULL) { // if it's a number
        alt = strtof(token, &end);
      }
    }
  }
  // Serial.print("Latitude part: ");
  // Serial.println(lat);
  // Serial.print("Longitude part: ");
  // Serial.println(lon);
  // Serial.print("Altitude part: ");
  // Serial.println(alt);
}

unsigned long getTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return 0;
  }
  // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  return timeinfo.tm_year + 1900;
}

void setup_wifi() {
  // TODO: 
  // setup wifi to sync local time and set unix time
  Serial.println("Starting setup");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // WiFi.mode(WIFI_STA);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("time set");
  // set unixtime
  unixtime = getTime();

  display.clearDisplay();
  display.println("Time set");
  display.display();
}

void clearBuffer() {
  for (int i = 0; i < count; i++) {
    gps[i] == NULL;
  }
}