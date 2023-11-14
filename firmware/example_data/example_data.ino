#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

unsigned char gps[64];
int count = 0; 

String sat_name = "ISS (ZARYA)";
unsigned char tle_1[70];
unsigned char tle_2[70];

int state = 2; // TODO: change starting state

void setup() {
    // setup gps
    // Serial1.begin(9600); // Serial for the GPS module, will need to define TX and RX for ESP32
    // setup motors here
    // set up serial connection to computer here
    Serial.begin(9600); // Starts the serial communication to the computer

    // setup screen
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

    display.drawPixel(10, 10, SSD1306_WHITE);

    Serial.println("Success");

    display.display();
    display.clearDisplay();

    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner

}

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
    if (state == 3) motor_track();
    // Step 4: send the motor data to the computer every 10 seconds
    if (state == 4) update_server();
}

void read_gps() {
  display.println(F("State 1"));
  if (Serial1.available()) {
      while (Serial1.available()) {
        gps[count++] = Serial1.read();
        if (count == 64) {
          break;
        }
      }
      Serial.write(gps, count);
      clearBuffer();
      count = 0;
    }
}

void process_tle() {
  if (Serial.available()) {
    display.println(F("State 2"));
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
    state = 4;
  }
}


void track_motor() {
  // TODO
}

void update_server() {
  if (Serial.available()) {
    display.println(F("State 4"));
    String input = Serial.readStringUntil('\n');
    if (input == "get_coords") {
      Serial.println(sat_name + "\n" + tle_1 + "\n" + tle_2); // Send actual coordinates here
    }
  }
}

void clearBuffer() {
  for (int i = 0; i < count; i++) {
    gps[i] == NULL;
  }
}