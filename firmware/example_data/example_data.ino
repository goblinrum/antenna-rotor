#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <Sgp4.h>
#include <Ticker.h>

// user Serial 2 for GPS
#define RXD2 16
#define TXD2 17

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

// TLE data
char satname[] = "ISS (ZARYA)";
char tle_1[70];
char tle_2[70];

// SGP4 information
Sgp4 sat;
Ticker tkSecond;
unsigned long unixtime = 1458950400;
int timezone = -8 ;  //utc - 8
int framerate;
int year; int mon; int day; int hr; int minute; double sec;
bool done_setup = false;

// current state machine state
int state = 3; // TODO: change starting state

void setup() {
    // set up serial connection to computer here
    Serial.begin(9600); // Starts the serial communication to the computer

    // setup gps serial
    Serial2.begin(9600); // Serial for the GPS module, will need to define TX and RX for ESP32
    // setup motors here

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

}
/*
State machine management on loop 
*/
void loop() {
    // Step 1: read the gps data (leave in acquring data mode until fix is found)
    if (state == 1) read_gps();
    // Step 2: 
    //  - Option 1: use the gps data to directly request API
    //  - Option 2: send GPS data to computer, and have computer request API
    if (state == 2)  process_tle();
    // Step 3: use the API data to calculate the position needed
    // uses library like this: https://github.com/Hopperpop/Sgp4-Library
    // this enters tracking mode
    if (state == 3) predict();
    // Step 4: Actually move the motors to the desired azimuth/elevation
    if (state == 4) track_motor();
    // Step 5: send the motor data to the computer every 10 seconds
    if (state == 5) update_server();
}
/* 
Read from GPS until a fix is acquired. 
Transitions into read_gps()
*/
void setup_gps() {
  // read from GPS
  if (Serial2.available()) {
    while (Serial2.available()) {
      gps[count++] = Serial2.read();
      if (count == 64) {
        break;
      }
    }

    Serial.write(gps, count);
    delay(1000);

    // Check for the substring "V,,,"
    if (strstr(gps, "V") != NULL && strstr(gps, "GPRMC") != NULL) {
      // Substring found meaning that fix is invalid
      clearBuffer();
    } else if (strstr(gps, "GPRMC") != NULL) {
      // Signal is fixed
      read_gps();
      state = 2;
    } else {
      // not checking the right line
      clearBuffer();
    }
    count = 0;
  }
}

/* 
Uses the GPS data and parses the $GPGGA field into
latitude, longitude, and altidude

Transitions into process_tle()
*/
void read_gps() {
  // TODO: remove test data
  char test_data[90] = "$GPGGA,234809.000,3752.4871,N,12215.4452,W,1,7,1.09,176.7,M,-24.9,M,,*6B";
  token = strtok(test_data, delimiter); // Initial tokenization
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
  Serial.print("Latitude part: ");
  Serial.println(lat);
  Serial.print("Longitude part: ");
  Serial.println(lon);
  Serial.print("Altitude part: ");
  Serial.println(alt);
  state = 2;
}

/* 
Process the TLE from the computer and saves it
Transitons to predict()
*/
void process_tle() {
  if (Serial.available()) {
    String input = Serial.readString();
    bool crossed = false;
    int indexForTle2 = 0;
    for (int i = 0; i < input.length(); i++) {
      if (input[i] == '\n') {
        crossed = true;
        tle_1[i] = '\0'; // Null-terminate tle_1
        continue; // Skip the newline character
      }
      if (!crossed) {
        tle_1[i] = input[i];
      } else {
        tle_2[indexForTle2++] = input[i];
      }
    }
    tle_2[indexForTle2] = '\0'; // Null-terminate tle_2
    state = 3;
  }
}

/* 
Performs first time setup if not done yet, then loops to do the actual prediction
*/
void predict() {

  if (done_setup) { // predict based on new unixtime
    sat.findsat(unixtime);
    framerate += 1;
  } else { // first time setup
    // TODO: remove test data
    char tle_line2[] = "2 25544  51.6430 274.9054 0000949 329.9492 173.3546 15.49459138426013";
    char tle_line1[] = "1 25544U 98067A   23324.37934042  .00011446  00000+0  21219-3 0  9999";
    lat = 37.52;
    lon = -122.15;
    alt = 176.70;
    sat.site(lat, lon, alt);
    sat.init(satname, tle_line1, tle_line2);
    double jdC = sat.satrec.jdsatepoch;
    invjday(jdC , timezone, true, year, mon, day, hr, minute, sec);
    Serial.println("Epoch: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
    Serial.println();
    tkSecond.attach(1,Second_Tick);
    done_setup = true;
  }
}

/* 
Moves the motor to input azimuth and elevation
*/
void track_motor() {
  // TODO
}

/*
Updates the server on the new position and coordinates 
*/
void update_server() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input == "get_coords") {
      Serial.println(satname); // Send actual coordinates here
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
  Serial.println(String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
  Serial.println("azimuth = " + String( sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
  Serial.println("latitude = " + String( sat.satLat) + " longitude = " + String( sat.satLon) + " altitude = " + String( sat.satAlt));

  switch(sat.satVis){
    case -2:
      Serial.println("Visible : Under horizon");
      break;
    case -1:
      Serial.println("Visible : Daylight");
      break;
    default:
      Serial.println("Visible : " + String(sat.satVis));   //0:eclipsed - 1000:visible
      break;
  }

  Serial.println("Framerate: " + String(framerate) + " calc/sec");
  Serial.println();
     
  framerate=0;
}

void clearBuffer() {
  for (int i = 0; i < count; i++) {
    gps[i] == NULL;
  }
}